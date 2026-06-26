// TensorRTDepthEngine — TRT-accelerated depth inference.
// Only compiled when DEDALUS_TENSORRT_ENABLED (CMake DEDALUS_TENSORRT=ON).

#ifdef DEDALUS_TENSORRT_ENABLED

#include "dedalus/sensing/tensorrt_depth_engine.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <cuda_runtime.h>
#include <NvInfer.h>
#include <NvOnnxParser.h>

namespace dedalus {
namespace {

// ---------------------------------------------------------------------------
// TRT logger — only kWARNING and above to stderr
// ---------------------------------------------------------------------------
class TrtLogger final : public nvinfer1::ILogger {
public:
    void log(Severity sev, const char* msg) noexcept override {
        if (sev <= Severity::kWARNING && msg) {
            fprintf(stderr, "[TRT] %s\n", msg);
        }
    }
};

// ---------------------------------------------------------------------------
// RAII wrappers for TRT objects (all use ->destroy())
// ---------------------------------------------------------------------------
template<typename T>
struct TrtDeleter { void operator()(T* p) const noexcept { if (p) p->destroy(); } };
template<typename T>
using TrtPtr = std::unique_ptr<T, TrtDeleter<T>>;

// ---------------------------------------------------------------------------
// Nearest-neighbour resize + ImageNet normalise → NCHW float tensor
// ---------------------------------------------------------------------------
std::vector<float> resize_and_normalise(
    const std::vector<std::uint8_t>& bytes,
    int src_w, int src_h, int dst_w, int dst_h,
    float mean_r, float mean_g, float mean_b,
    float std_r,  float std_g,  float std_b)
{
    std::vector<float> tensor(static_cast<std::size_t>(3 * dst_h * dst_w), 0.0F);
    const float sx = static_cast<float>(src_w) / static_cast<float>(dst_w);
    const float sy = static_cast<float>(src_h) / static_cast<float>(dst_h);
    const float mean[3] = {mean_r, mean_g, mean_b};
    const float stdv[3] = {std_r,  std_g,  std_b};

    for (int dy = 0; dy < dst_h; ++dy) {
        const int sy_i = std::min(static_cast<int>(dy * sy), src_h - 1);
        for (int dx = 0; dx < dst_w; ++dx) {
            const int sx_i = std::min(static_cast<int>(dx * sx), src_w - 1);
            const std::size_t src_idx =
                (static_cast<std::size_t>(sy_i) * static_cast<std::size_t>(src_w) +
                 static_cast<std::size_t>(sx_i)) * 3u;
            for (int c = 0; c < 3; ++c) {
                const float pixel = static_cast<float>(bytes[src_idx + c]) / 255.0F;
                const float norm  = (pixel - mean[c]) / stdv[c];
                const std::size_t dst_idx =
                    static_cast<std::size_t>(c * dst_h * dst_w + dy * dst_w + dx);
                tensor[dst_idx] = norm;
            }
        }
    }
    return tensor;
}

}  // namespace

// ---------------------------------------------------------------------------
// Impl
// ---------------------------------------------------------------------------
struct TensorRTDepthEngine::Impl {
    TensorRTDepthEngineConfig config;
    TrtLogger                 logger;

    TrtPtr<nvinfer1::IRuntime>          runtime;
    TrtPtr<nvinfer1::ICudaEngine>       engine;
    TrtPtr<nvinfer1::IExecutionContext> context;

    cudaStream_t stream{nullptr};

    void*       d_input{nullptr};
    void*       d_output{nullptr};
    std::size_t input_bytes{0};
    std::size_t output_bytes{0};

    int input_binding{-1};
    int output_binding{-1};
    int output_h{0};
    int output_w{0};

    explicit Impl(TensorRTDepthEngineConfig cfg) : config(std::move(cfg)) {
        cudaSetDevice(config.device_id);
        cudaStreamCreate(&stream);

        runtime.reset(nvinfer1::createInferRuntime(logger));
        if (!runtime) throw std::runtime_error("TRT: createInferRuntime failed");

        const std::filesystem::path model(config.engine_path);
        const std::string ext = model.extension().string();

        if (ext == ".engine") {
            load_engine(model);
        } else if (ext == ".onnx") {
            const std::filesystem::path cache(config.engine_path + ".trt_cache");
            if (std::filesystem::exists(cache)) {
                load_engine(cache);
            } else {
                build_engine(model, cache);
            }
        } else {
            throw std::runtime_error("TRT: unsupported model extension: " + ext);
        }

        setup_bindings();
        alloc_device_buffers();
    }

    ~Impl() {
        cudaStreamSynchronize(stream);
        cudaStreamDestroy(stream);
        cudaFree(d_input);
        cudaFree(d_output);
    }

    void load_engine(const std::filesystem::path& path) {
        std::ifstream f(path, std::ios::binary);
        if (!f) throw std::runtime_error("TRT: cannot open " + path.string());
        std::vector<char> data((std::istreambuf_iterator<char>(f)), {});
        engine.reset(runtime->deserializeCudaEngine(data.data(), data.size()));
        if (!engine) throw std::runtime_error("TRT: deserializeCudaEngine failed");
    }

    void build_engine(const std::filesystem::path& onnx_path,
                      const std::filesystem::path& cache_path) {
        TrtPtr<nvinfer1::IBuilder> builder{nvinfer1::createInferBuilder(logger)};
        if (!builder) throw std::runtime_error("TRT: createInferBuilder failed");

        const auto flags = 1u << static_cast<unsigned>(
            nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
        TrtPtr<nvinfer1::INetworkDefinition> net{builder->createNetworkV2(flags)};

        TrtPtr<nvonnxparser::IParser> parser{nvonnxparser::createParser(*net, logger)};
        if (!parser->parseFromFile(onnx_path.c_str(),
                static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
            throw std::runtime_error("TRT: ONNX parse failed: " + onnx_path.string());
        }

        TrtPtr<nvinfer1::IBuilderConfig> bcfg{builder->createBuilderConfig()};
        // Limit workspace to avoid competing with AirSim VRAM (~3 GiB)
        bcfg->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE,
                                 config.builder_workspace_bytes);
        if (config.use_fp16 && builder->platformHasFastFp16()) {
            bcfg->setFlag(nvinfer1::BuilderFlag::kFP16);
        }

        TrtPtr<nvinfer1::IHostMemory> plan{builder->buildSerializedNetwork(*net, *bcfg)};
        if (!plan) throw std::runtime_error("TRT: buildSerializedNetwork failed");

        std::ofstream cache(cache_path, std::ios::binary);
        cache.write(static_cast<const char*>(plan->data()),
                    static_cast<std::streamsize>(plan->size()));

        engine.reset(runtime->deserializeCudaEngine(plan->data(), plan->size()));
        if (!engine) throw std::runtime_error("TRT: deserialize after build failed");
    }

    void setup_bindings() {
        context.reset(engine->createExecutionContext());
        if (!context) throw std::runtime_error("TRT: createExecutionContext failed");

        const int n = engine->getNbBindings();
        for (int i = 0; i < n; ++i) {
            if (engine->bindingIsInput(i)) input_binding  = i;
            else                           output_binding = i;
        }
        if (input_binding < 0 || output_binding < 0) {
            throw std::runtime_error("TRT: could not find input/output bindings");
        }

        const auto out_dims = engine->getBindingDimensions(output_binding);
        output_h = out_dims.d[out_dims.nbDims - 2];
        output_w = out_dims.d[out_dims.nbDims - 1];
    }

    void alloc_device_buffers() {
        input_bytes  = static_cast<std::size_t>(3 * config.model_input_height
                                                  * config.model_input_width) * sizeof(float);
        output_bytes = static_cast<std::size_t>(output_h * output_w) * sizeof(float);
        cudaMalloc(&d_input,  input_bytes);
        cudaMalloc(&d_output, output_bytes);
    }

    DepthInferenceResult run(const VisualDepthFrame& frame) {
        DepthInferenceResult result;
        result.width  = output_w;
        result.height = output_h;
        if (frame.bytes.empty() || frame.width <= 0 || frame.height <= 0) return result;

        const auto t0 = std::chrono::steady_clock::now();

        std::vector<float> tensor = resize_and_normalise(
            frame.bytes, frame.width, frame.height,
            config.model_input_width, config.model_input_height,
            config.mean_r, config.mean_g, config.mean_b,
            config.std_r,  config.std_g,  config.std_b);

        cudaMemcpyAsync(d_input, tensor.data(), input_bytes,
                        cudaMemcpyHostToDevice, stream);

        void* bindings[2] = {};
        bindings[input_binding]  = d_input;
        bindings[output_binding] = d_output;

        // TRT 8.x: enqueueV2; TRT 10.x: executeV2
#if NV_TENSORRT_MAJOR >= 10
        context->executeV2(bindings);
#else
        context->enqueueV2(bindings, stream, nullptr);
#endif

        result.depth_relative.resize(static_cast<std::size_t>(output_h * output_w));
        cudaMemcpyAsync(result.depth_relative.data(), d_output, output_bytes,
                        cudaMemcpyDeviceToHost, stream);
        cudaStreamSynchronize(stream);

        // Clamp to valid disparity range (model may produce near-zero on sky pixels)
        for (float& v : result.depth_relative) {
            if (v <= 0.0F || !std::isfinite(v)) v = 1.0e-4F;
        }

        const auto t1 = std::chrono::steady_clock::now();
        result.inference_time_ms = static_cast<float>(
            std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count()) / 1000.0F;
        result.valid = true;
        return result;
    }
};

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
TensorRTDepthEngine::TensorRTDepthEngine(TensorRTDepthEngineConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

TensorRTDepthEngine::~TensorRTDepthEngine() = default;

DepthInferenceResult TensorRTDepthEngine::infer(const VisualDepthFrame& frame) {
    return impl_->run(frame);
}

std::string TensorRTDepthEngine::engine_name() const {
    return "tensorrt_depth_engine";
}

}  // namespace dedalus

#endif  // DEDALUS_TENSORRT_ENABLED
