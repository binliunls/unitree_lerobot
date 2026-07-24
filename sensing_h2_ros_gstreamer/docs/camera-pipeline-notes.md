# Camera Pipeline Notes

Use this page when debugging the implementation. For normal bring-up, start with
`docs/camera-bringup.md`.

## Runtime Shape

Each camera runs as its own `gscam2` process. Each process owns one GStreamer
pipeline and publishes a standard `image_raw` and `camera_info` pair.

The launch helpers set the GStreamer pipeline through `GSCAM_CONFIG`, source the
local ROS 2 environment, and default to CycloneDDS for image-heavy topics.

## Head Pipeline

The head cameras use `nvarguscamerasrc` with Argus sensor IDs `0` and `1`.

The active head path captures calibrated `2560x1984` from Argus, downscales to `640x480` in
`nvvidconv`, converts the Argus NVMM buffer into CPU-readable `NV12`, and then
uses `videoconvert` for final packed `RGB`.

`videoconvert` cannot consume Argus `memory:NVMM` directly, so `nvvidconv` is the
required handoff from Argus/NVMM into CPU-readable video buffers.

## Wrist Pipeline

The wrist cameras use V4L2 devices `/dev/video2` and `/dev/video3`.

The active wrist path captures calibrated `UYVY 1920x1536`, scales to `640x480`,
and uses CPU `videoconvert` for final packed `RGB`.

This keeps the published wrist image size aligned with the head cameras.

## Why The Final Convert Remains

`nvvidconv` does not produce packed `RGB` on this target. It can produce formats
such as `RGBA` and `BGRx`, but the ROS output contract is `rgb8`, so the active
paths still need a final `videoconvert` step before `gscam2` publishes.

## Latency And Timestamps

The launch helpers default `sync_sink:=false`. That favors low latency and
latest-frame delivery for realtime robotics streams instead of blocking upstream
to preserve every frame in media-clock order.

The launch helpers also set `use_gst_timestamps:=false`, so ROS message headers
use ROS publish time. This is the current bring-up contract; use calibrated
hardware or synchronized timestamps separately if consumers require them.

The pipeline queues are leaky downstream with a one-buffer limit, which keeps
the streams from accumulating stale frames under transient load.

## Debug Tracing

Set `DEBUG_LOG` to a nonzero value before running a launch helper to enable
GStreamer debug/tracer output under `/tmp/gscam2-traces/`.

Useful overrides are `GSCAM2_TRACE_DIR`, `GST_TRACERS`, `GST_DEBUG`, and
`GST_DEBUG_FILE`. The default tracer is `latency`; add `rusage` and `stats` only
when you need noisier profiling output.

## Accelerated Wrist Status

The accelerated wrist path is deferred. It reduced single-camera userspace CPU
in testing, but under the `gscam2` wrist path it showed visual artifacts,
including top-left crop and horizontal tearing.

The CPU `videoconvert` wrist path is the active path until that accelerated
handoff is debugged further.
