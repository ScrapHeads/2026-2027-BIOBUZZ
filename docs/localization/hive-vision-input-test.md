# HIVE vision input check

Run **TEST - Hive Vision Inputs** from the Driver Station TeleOp list. The hardware configuration must name the Limelight `limelight`. This standalone OpMode initializes only Limelight, so position the robot manually.

The test checks inputs that affect the proposed HIVE correction. It does not implement correction, fuse a pose, determine capture time, upload a map, or validate camera firmware in general.

## Setup

1. Load the intended static flat-HIVE map and select AprilTag processing in Limelight.
2. Confirm **Snap Robot to Floor is OFF** in that pipeline.
3. In Dashboard configuration, set `HiveVisionInputTest.PIPELINE` before pressing INIT. It defaults to the project's AprilTag pipeline. The test sends one pipeline request at initialization; changing the field afterward requires restarting the OpMode.
4. Keep tags on only the intended HIVE in view where possible, press START, and check returned pipeline, validity, positive pose tag count, and finite raw MT1 values.
5. Press **A** to capture a fresh valid baseline. Press **X** to clear it. Repeat A after moving to a new test configuration. A detected camera timestamp regression also clears the baseline.

## Checks that affect implementation

| Check | Physical procedure | What to record |
|---|---|---|
| Height is available | Keep the robot level and stationary, capture a baseline with the HIVE flat, then tilt the HIVE by a measured angle. Repeat at multiple positions. | Raw X/Y/Z and roll/pitch/yaw before and after; actual HIVE angle and robot location. Z pinned to zero across varied tilted configurations suggests floor snapping or a degenerate observation. One near-zero Z value does not establish failure. |
| Rotation convention | With the HIVE flat, capture a baseline and manually rotate the level robot in a known direction. Repeat while looking at a tilted HIVE from several headings, including roughly 90 degrees to the tilt plane. | Actual robot heading and raw roll/pitch/yaw signs. Compare against the specified forward rotation model once the solver is implemented; do not assume raw pitch equals HIVE tilt. |
| Field frame and units | With the HIVE flat, move the robot a measured distance along each chosen field axis, keeping heading fixed. Capture a fresh baseline for each move. | Raw position deltas in meters and heading in degrees. Establish signs, scale, and origin relative to existing Pedro paths. Raw tilted-HIVE poses are intentionally distorted and should not be compared directly with true robot position. |
| Frame identity | Leave the robot stationary; observe new and repeated frame counts, then temporarily obscure the tags. Optionally restart the camera. | Camera timestamp changes, gaps since a new timestamp, validity, and backwards-timestamp count. Repeated reads are normal. A frozen timestamp must not be fused repeatedly, even when SDK receipt age is small. |
| Required timing fields | Observe capture, processing, and parse latency, receipt age, and the camera timestamp. | Missing/nonfinite/negative fields and typical values. Capture plus processing latency is not a proven end-to-end capture age; transport/polling delay remains to be characterized. |
| HIVE/pipeline selection | Start separately with each configured localization pipeline, looking at known red or blue tags. If necessary, test visibility of both HIVES. | Returned pipeline, detected IDs, pose tag count, and raw pose. Mixed detected HIVE groups are a rejection warning. Detection IDs do not identify the exact MT1 contributing set or prove which map is loaded. Verify maps in the camera configuration. |

## Interpreting telemetry

- Positions are meters; displayed rotations and wrapped angle differences are degrees.
- `Pose payload / freshness checks: PASS` only means a valid result, a six-value finite MT1 pose, positive pose tag count, matching pipeline, and recent camera timestamp activity. It does **not** certify geometry, map identity, HIVE angle, or fusion readiness.
- `MAX_FRAME_AGE_MS` defaults to 250 ms for diagnosis and is configurable. This is not a competition fusion threshold.
- SDK receipt age uses wall time; elapsed time since a new camera timestamp uses a monotonic local clock. Neither is presented as a synchronized capture timestamp.
- Camera timestamp is displayed in its native clock without assuming its units or subtracting it from the local clock.
- Latency-field status is reported separately from pose-shape status.
- Baseline comparisons are marked invalid/stale when the current frame fails checks. No baseline is captured from such a frame.
- Zero X/Y is allowed. Zero Z is informational, not automatically rejected.

Report measured inputs and telemetry discrepancies before changing mathematical signs, field transforms, or calibration. Mathematical root selection, singularities, and inverse-transform correctness belong in deterministic tests when the pure solver is implemented.
