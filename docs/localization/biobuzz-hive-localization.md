# BIOBUZZ HIVE localization

## Implemented flow

`VisionSubsystem` owns Limelight and obtains full 3D MegaTag1 (`botpose`) snapshots from the SDK's background polling. `DrivetrainLocalizer` updates Pinpoint, submits odometry to the existing `RobotState` / RilLib `PoseEstimator`, processes one cached vision snapshot, then returns the fused pose to Pedro.

No second estimator or dynamic map upload is used. Missing camera, unknown alliance, invalid frames, wrong/mixed HIVE IDs, unavailable capture-time history, ill-conditioned or ambiguous angles, and innovation outliers produce odometry-only operation.

## Coordinates and mathematics

- RilLib and RobotState: meters and radians.
- Pedro and configured Pinpoint output: inches and radians, using the same physical field axes/origin as RobotState. Existing `ConversionUtil` handles scale only.
- MT1 adapter: position in meters and raw roll/pitch/yaw in degrees; orientation is converted into RilLib Rotation3d.
- Camera mounting extrinsics are applied by Limelight. They must not be applied twice in robot code.
- Each HIVE has a map-frame pivot X/Y/height and axis yaw. Local +Y is the pivot axis rotated around map +Z by that yaw.
- Map-to-canonical-field translation and yaw are independent of the pivot. They are applied after correction; alliance changes never mirror the odometry frame.

In the local pivot frame, solve `xRaw*sin(phi) + (zRaw-h)*cos(phi) = robotOriginHeight-h`, where `phi = -hiveAngle`. Both roots are evaluated. Correct the position and full rotation using the inverse pivot transform. The angle between corrected robot up and field up validates both roll and pitch, including headings where a pitch-only check loses information.

The derivative magnitude of the height constraint gives a conditioning gate. Near-equivalent surviving roots reject rather than defaulting to the smallest root. Corrected Z satisfies the floor equation by construction and is not treated as independent proof of measurement quality.

## Configuration and remaining TODOs

Configure `HiveLocalizationConfig` before INIT and restart the OpMode after edits. `ENABLED` can be toggled live to disable/re-enable fusion. The defaults allow operation under the agreed assumption that input verification succeeds, but the following constants are explicitly provisional:

1. Red pipeline 0 and blue pipeline 1, and their static-map assignments.
2. The reference pivot height of 43.95 inches, each HIVE pivot location/axis, and 35-degree mechanical window.
3. Both map-to-existing-Pedro-field transforms, currently identity.
4. Field bounds, currently +/-2.2 meters; change them if the team's canonical field uses a corner origin.
5. Position/heading standard deviations and innovation thresholds.
6. The approximate 20 ms transport delay.

These points have `// TODO` comments in source. Use the two diagnostic TeleOps to measure them. In each AprilTag pipeline, disable Snap Robot to Floor, configure measured camera mounting, and restrict the map/filter to the intended HIVE. No code here verifies the firmware's map association or changes the camera's map.

A single MT1 aggregate from independently tilted red and blue HIVES cannot be corrected by one rigid rotation. The code conservatively rejects detected IDs outside the selected HIVE. Because detected IDs are not necessarily the exact MT1 contributor set, camera-side map/filter configuration remains necessary.

Changing alliance reconfigures the pipeline once. If the request fails, restart the OpMode after resolving the connection; the loop does not repeatedly send network requests. An unknown alliance intentionally stays odometry-only. Manual switching to the game-element pipeline suspends localization until a localization pipeline is requested again.

## Timestamps, confidence, and reset

Localization uses `LocalizationClock.now()` (monotonic seconds). Legacy TimeTracker remains for unrelated features.

Camera timestamps are identity tokens only. Repeated snapshots are consumed once; camera timestamp regression discards that frame. Capture time is approximated from the paired local read time, SDK receipt age, capture latency, targeting latency, and configurable transport delay. Wall-clock discontinuities reject observations. This is an approximation, not hardware clock synchronization.

The localizer gates observations against the predicted pose at capture time. RobotState enforces actual history bounds, rejecting future, pre-reset, duplicate, and out-of-order measurements. It does not rely on PoseEstimator.sampleAt's endpoint clamping.

A bounded uncertainty heuristic accounts for tag count, distance, orientation residual, and solver conditioning. Missing current-frame quality is rejected; prior quality is never reused. Skew is not treated as tag ambiguity.

Pose resets clear vision/history. RobotState generation changes cause the Pinpoint bridge to adopt external reset/handoff origins. Auto now saves the fused meters/radians snapshot once at shutdown; TeleOp restores the same pose into both Pinpoint and RobotState. Vision corrections do not reset Pinpoint. Raw velocity is rotated into the corrected field heading rather than differentiating vision pose jumps.

## On-robot validation

1. Run **TEST - Hive Vision Inputs** using the procedure in `hive-vision-input-test.md`.
2. Set geometry and field transforms in `HiveLocalizationConfig`.
3. Run **TEST - Hive Localization**. Configure BLUE and surveyed START_X_IN / START_Y_IN / START_HEADING_DEG before INIT. It initializes Pinpoint and Limelight, with no drive motors.
4. Move the robot by hand. Compare raw odometry, corrected vision, fused pose, angle residual, rejection status, and accepted frame count. Press A only when physically back at the configured START pose to reset the estimator.
5. Tilt the selected HIVE while keeping the robot stationary; the corrected/fused robot pose should remain consistent with the survey.
6. Obscure tags or disconnect Limelight while moving manually. Pinpoint and fused odometry propagation must continue and accepted count must stop.
7. Restore tracking. Only valid fresh observations within the configured capture-time innovation gates may resume correction. Large accumulated errors require an explicit reset; there is no automatic large-jump relocalization.

HIVE temporal state/CELL selection and shooter targeting are intentionally outside this base localization implementation. The most recent frame's angle result is diagnostic and must not be treated as a persistent targeting state.

## Automated validation

The deterministic JVM tests cover an independent Rodrigues forward HIVE model, both tilt signs, heading wrap, sideways headings, offset pivots/axis yaw, nonzero robot origin height, impossible/ambiguous/degenerate roots, bad orientation, physical-frame deduplication, clock restart, latency conversion, historical fusion, and reset.

Targeted Gradle command:

`gradlew.bat :TeamCode:testDebugUnitTest --tests org.firstinspires.ftc.teamcode.robotTests.localization.* --tests org.firstinspires.ftc.teamcode.robotTests.RobotStateTest`

Run `:TeamCode:assembleDebug` to verify packaging. The full existing JVM suite currently also contains Dashboard mock tests that require the unavailable native RobotCore library on this desktop; those are separate from the deterministic localization tests.
