#include "dedalus/sensors/frame_source.hpp"

#include <future>
#include <memory>
#include <optional>
#include <utility>

namespace dedalus {

struct AsyncPrefetchFrameSource::Impl {
    std::unique_ptr<FrameSource> inner;
    std::future<std::optional<FramePacket>> prefetched;

    void launch_prefetch() {
        prefetched = std::async(std::launch::async, [this]() {
            return inner->next_frame();
        });
    }
};

AsyncPrefetchFrameSource::AsyncPrefetchFrameSource(std::unique_ptr<FrameSource> inner)
    : impl_(std::make_unique<Impl>()) {
    impl_->inner = std::move(inner);
    // Do NOT launch a prefetch here.  Constructing a FrameSource should be
    // side-effect free (no subprocesses, no connections).  The first prefetch
    // is launched lazily on the first next_frame() call so that code which
    // creates providers without calling next_frame() (e.g. config tests) is
    // not affected.
}

AsyncPrefetchFrameSource::~AsyncPrefetchFrameSource() {
    if (impl_->prefetched.valid()) {
        try {
            (void)impl_->prefetched.get();
        } catch (...) {
            // Speculative prefetch may still be waiting on I/O when the source is
            // destroyed (e.g. after a bounded --max-frames run).  Swallow the error
            // here; the caller has already finished processing all wanted frames.
        }
    }
}

std::optional<FramePacket> AsyncPrefetchFrameSource::next_frame() {
    if (!impl_->prefetched.valid()) {
        // First call: no prefetch in flight yet — fetch synchronously so the
        // caller gets frame 0 without delay, then immediately queue frame 1.
        impl_->launch_prefetch();
    }
    auto frame = impl_->prefetched.get();
    if (frame.has_value()) {
        // Overlap I/O for the next frame with processing of this one.
        impl_->launch_prefetch();
    }
    return frame;
}

void AsyncPrefetchFrameSource::request_stop() {
    impl_->inner->request_stop();
}

}  // namespace dedalus
