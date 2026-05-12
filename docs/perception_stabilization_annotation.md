# Perception Stabilization and Frame Annotation Hooks

This document describes two extension points added to the core perception/world-model path.

## Camera Stabilization Stage

The perception pipeline now runs:

```text
FramePacket
  -> Detector
  -> CameraStabilizer
  -> Tracker
  -> IdentityResolver
  -> Projector3D
  -> PerceptionPipelineOutput
```

The stabilizer is placed after detection and before tracking so it can use detected visual features to estimate frame-to-frame camera motion, then adjust the frame and/or detections before tracks are updated.

Current provider:

```yaml
camera_stabilizer: null
```

`NullCameraStabilizer` is a pass-through provider. It copies the input frame and detections unchanged and reports no transform.

Future providers can use detections, keypoints, optical flow, homography, IMU hints, or world-model landmarks to estimate stabilization transforms.

## Post-World-Model Annotation Stage

The runtime invokes a frame annotation sink after world-model ingestion:

```text
FramePacket + PerceptionPipelineOutput + WorldSnapshot
  -> FrameAnnotationSink
```

The CI-safe default remains:

```yaml
frame_annotator: null
```

`NullFrameAnnotationSink` is a no-op and remains the default for normal unit tests and smoke paths.

A dependency-light visual validation provider is available:

```yaml
frame_annotator: ppm_sequence
annotation_output_path: out/annotations
annotation_output_fps: 5
```

`PpmFrameAnnotationSink` copies the RGB frame, draws simple debug overlays with raw pixel operations, and writes an image sequence:

```text
out/annotations/frame_000001.ppm
out/annotations/frame_000002.ppm
...
out/annotations/manifest.txt
```

The manifest records frame index, frame ID, timestamp, path, and configured output FPS. The PPM sequence provider intentionally avoids OpenCV, FFmpeg, GStreamer, and media-codec dependencies so it can run inside the existing C++ unit-test environment.

The first overlay slice includes:

```text
detection boxes
track boxes
track IDs
class labels
frame ID
frame timestamp
active map frame
ego map frame
simple tactical exclusion zone markers
simple flight corridor markers
simple landmark markers
```

An explicit placeholder still exists for future MP4 export:

```yaml
frame_annotator: mp4
annotation_output_path: out/annotated.mp4
annotation_output_fps: 5
```

The MP4 sink intentionally fails today with a clear not-implemented error. A real implementation should either depend on an optional external encoder path or live behind a separate optional provider so OpenCV, FFmpeg, and GStreamer do not become mandatory for core tests.

## Side-by-Side Validation Goal

The intended validation path is:

```text
raw input frames
  -> processed FPS annotation/export
  -> annotated image sequence or video with the same mission duration as the raw sensor input
```

For side-by-side viewing, the annotation exporter should preserve input-time semantics:

```text
output duration == raw input duration
output FPS == configured processing/annotation FPS
frame timestamps drive video timing or frame duplication/drop policy
```

The PPM sequence provider records timestamps and configured FPS in its manifest. A later MP4/image-sequence-to-video slice can use that manifest to preserve mission timing and implement duplication/drop policy without changing the core `FrameAnnotationSink` contract.

## Provider Names

Current registry entries:

```text
camera_stabilizer: null
frame_annotator: null, ppm_sequence, mp4
```

Keep future stabilizers and annotators behind these provider interfaces. Do not hardcode OpenCV, FFmpeg, GStreamer, camera SDK, or UI dependencies into the core perception/world-model contracts.
