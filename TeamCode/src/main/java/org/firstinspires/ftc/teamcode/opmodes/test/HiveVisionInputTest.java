package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.VisionConstants;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.List;

/**
 * Checks the actual MT1 inputs needed by the HIVE correction, without moving
 * motors, uploading maps, or submitting observations to RobotState.
 * See docs/localization/hive-vision-input-test.md for the physical procedure.
 */
@Config
@TeleOp(name = "TEST - Hive Vision Inputs", group = "Test")
public class HiveVisionInputTest extends LinearOpMode {
    // Configure before INIT. Changing this field while running does not switch pipelines.
    public static int PIPELINE = VisionConstants.APRIL_TAG_PIPELINE;
    public static long MAX_FRAME_AGE_MS = 250;

    private double[] baseline;
    private double lastCameraTimestamp = Double.NaN;
    private long lastNewFrameNanos;
    private long newFrames;
    private long repeatedReads;
    private long timestampRegressions;
    private boolean previousA;
    private boolean previousX;

    @Override
    public void runOpMode() throws InterruptedException {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        final int requestedPipeline = PIPELINE;
        if (requestedPipeline < 0 || requestedPipeline > 9) {
            telemetry.addLine("PIPELINE must be between 0 and 9. Change it before INIT.");
            telemetry.update();
            return;
        }

        limelight.setPollRateHz(50);
        if (!limelight.pipelineSwitch(requestedPipeline)) {
            telemetry.addLine("Pipeline request failed. Check the camera and restart this test.");
            telemetry.update();
            return;
        }

        try {
            limelight.start();
            telemetry.setMsTransmissionInterval(100);
            telemetry.addLine("Static flat-HIVE map; Snap Robot to Floor must be OFF.");
            telemetry.addLine("START, then A: capture baseline; X: clear baseline.");
            telemetry.addLine("Only Limelight is initialized. Move robot/HIVE manually.");
            telemetry.update();
            waitForStart();

            while (opModeIsActive()) {
                boolean capture = gamepad1.a && !previousA;
                boolean clear = gamepad1.x && !previousX;
                previousA = gamepad1.a;
                previousX = gamepad1.x;
                if (clear) baseline = null;

                telemetry.addData("Requested pipeline", requestedPipeline);
                telemetry.addLine("A: baseline | X: clear | position: m | angles: deg");
                LLResult result = limelight.getLatestResult();
                if (result == null) {
                    telemetry.addLine("No result received.");
                } else {
                    report(result, requestedPipeline, capture);
                }
                telemetry.update();
                sleep(20);
            }
        } finally {
            limelight.stop();
        }
    }

    private void report(LLResult result, int requestedPipeline, boolean capture) {
        try {
            // SDK getters can return zero defaults for missing fields. Check the
            // actual payload before treating zero as an observed input.
            JSONObject json = new JSONObject(result.toString());
            double timestamp = json.optDouble("ts", Double.NaN);
            int pipeline = json.optInt("pID", -1);
            int tagCount = json.optInt("botpose_tagcount", -1);
            double captureMs = json.optDouble("cl", Double.NaN);
            double processingMs = json.optDouble("tl", Double.NaN);
            long now = System.nanoTime();

            boolean hasTimestamp = Double.isFinite(timestamp) && timestamp > 0;
            boolean isNew = hasTimestamp && timestamp != lastCameraTimestamp;
            boolean regressed = hasTimestamp && Double.isFinite(lastCameraTimestamp)
                    && timestamp < lastCameraTimestamp;
            if (isNew) {
                if (regressed) {
                    timestampRegressions++;
                    baseline = null;
                }
                lastCameraTimestamp = timestamp;
                lastNewFrameNanos = now;
                newFrames++;
            } else if (hasTimestamp) {
                repeatedReads++;
            }

            long localAgeMs = lastNewFrameNanos == 0
                    ? Long.MAX_VALUE : (now - lastNewFrameNanos) / 1_000_000;
            long sdkAgeMs = result.getStaleness();
            long ageLimit = Math.max(1, MAX_FRAME_AGE_MS);
            boolean fresh = hasTimestamp && localAgeMs <= ageLimit
                    && sdkAgeMs >= 0 && sdkAgeMs <= ageLimit && !regressed;
            telemetry.addData("Returned pipeline", pipeline);
            telemetry.addData("Valid / pose tag count", "%s / %d", result.isValid(), tagCount);
            telemetry.addData("Camera ts (camera clock)", "%.6f", timestamp);
            telemetry.addData("New / repeated / backwards", "%d / %d / %d",
                    newFrames, repeatedReads, timestampRegressions);
            telemetry.addData("Age since new camera frame (ms)", localAgeMs);
            telemetry.addData("SDK receipt age (ms)", sdkAgeMs);
            telemetry.addData("Capture / processing / parse (ms)", "%.2f / %.2f / %.2f",
                    captureMs, processingMs, result.getParseLatency());
            telemetry.addLine("Receipt age is not capture age; camera ts is a different clock.");

            reportTags(result.getFiducialResults());
            double[] raw = readPose(json.optJSONArray("botpose"));
            boolean poseUsable = result.isValid() && pipeline == requestedPipeline
                    && tagCount > 0 && raw != null && fresh;
            telemetry.addData("Pose payload / freshness checks",
                    poseUsable ? "PASS (geometry still needs physical checks)" : "NOT READY");
            telemetry.addData("Latency fields present and nonnegative",
                    Double.isFinite(captureMs) && captureMs >= 0
                            && Double.isFinite(processingMs) && processingMs >= 0);
            telemetry.addData("MT1 average distance (m)",
                    json.optDouble("botpose_avgdist", Double.NaN));

            if (raw == null) {
                telemetry.addLine("Missing, short, or nonfinite MT1 botpose array.");
                return;
            }
            telemetry.addData("Raw MT1 X / Y / Z (m)", "%.4f / %.4f / %.4f",
                    raw[0], raw[1], raw[2]);
            telemetry.addData("Raw roll / pitch / yaw (deg)", "%.2f / %.2f / %.2f",
                    raw[3], raw[4], raw[5]);

            if (capture) {
                if (poseUsable) baseline = raw.clone();
                else telemetry.addLine("Baseline refused: obtain a fresh valid MT1 pose first.");
            }
            if (baseline != null) {
                telemetry.addData("Delta X / Y / Z (m)", "%.4f / %.4f / %.4f",
                        raw[0] - baseline[0], raw[1] - baseline[1], raw[2] - baseline[2]);
                telemetry.addData("Delta roll / pitch / yaw (deg)", "%.2f / %.2f / %.2f",
                        wrapDegrees(raw[3] - baseline[3]), wrapDegrees(raw[4] - baseline[4]),
                        wrapDegrees(raw[5] - baseline[5]));
                telemetry.addData("Comparison data", poseUsable ? "CURRENT" : "INVALID OR STALE");
            }
            if (Math.abs(raw[2]) < 0.001) {
                telemetry.addLine("Z near zero: valid when flat; check OFF-floor-snap with HIVE tilted.");
            }
        } catch (JSONException e) {
            telemetry.addLine("Could not parse the current result payload; no baseline captured.");
        }
    }

    private void reportTags(List<LLResultTypes.FiducialResult> tags) {
        boolean red = false;
        boolean blue = false;
        boolean unknown = false;
        StringBuilder ids = new StringBuilder();
        if (tags != null) {
            for (LLResultTypes.FiducialResult tag : tags) {
                int id = tag.getFiducialId();
                if (ids.length() > 0) ids.append(", ");
                ids.append(id);
                if (id >= 30 && id <= 37) red = true;
                else if (id >= 38 && id <= 45) blue = true;
                else unknown = true;
            }
        }
        telemetry.addData("Detected tag IDs", ids.length() == 0 ? "none" : ids);
        telemetry.addData("Detected HIVE group",
                red && blue ? "MIXED: do not fuse an aggregate pose"
                        : unknown ? "Includes unexpected IDs"
                        : red ? "RED" : blue ? "BLUE" : "UNKNOWN");
        telemetry.addLine("Detected IDs alone do not prove which tags MT1 used.");
    }

    private static double[] readPose(JSONArray array) {
        if (array == null || array.length() < 6) return null;
        double[] pose = new double[6];
        for (int i = 0; i < pose.length; i++) {
            pose[i] = array.optDouble(i, Double.NaN);
            if (!Double.isFinite(pose[i])) return null;
        }
        return pose;
    }

    private static double wrapDegrees(double degrees) {
        return Math.toDegrees(Math.atan2(Math.sin(Math.toRadians(degrees)),
                Math.cos(Math.toRadians(degrees))));
    }
}
