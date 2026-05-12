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

The runtime now invokes a frame annotation sink after world-model ingestion:

```text
FramePacket + PerceptionPipelineOutput + WorldSnapshot
  -> FrameAnnotationSink
```

Current provider:

```yaml
frame_annotator: null
```

`NullFrameAnnotationSink` is a no-op and is CI-safe.

An explicit placeholder exists for future MP4 export:

```yaml
frame_annotator: mp4
annotation_output_path: out/annotated.mp4
annotation_output_fps: 5
```

The MP4 sink intentionally fails today with a clear not-implemented error. A real implementation should render world-model state back onto stabilized frames and write an annotated video stream.

## Side-by-Side Validation Goal

The intended future validation path is:

```text
raw input frames
  -> processed FPS annotation/export
  -> annotated MP4 with the same wall-clock duration as the raw sensor input
```

For side-by-side viewing, the annotation exporter should preserve input-time semantics:

```text
output duration == raw input duration
output FPS == configured processing/annotation FPS
frame timestamps drive video timing or frame duplication/drop policy
```

This allows raw and annotated streams to be compared visually without changing the mission timeline.

## Provider Names

Current registry entries:

```text
camera_stabilizer: null
frame_annotator: null, mp4
```

Keep future stabilizers and annotators behind these provider interfaces. Do not hardcode OpenCV, FFmpeg, GStreamer, or UI dependencies into the core perception/world-model contracts.
