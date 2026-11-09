package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.VisionConstants.*;
import static org.firstinspires.ftc.teamcode.vision.tracking.TrackedGameElement.calcElementType;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose3d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation3d;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.config.HiveLocalizationConfig;
import org.firstinspires.ftc.teamcode.util.LocalizationClock;
import org.firstinspires.ftc.teamcode.util.TimeTracker;
import org.firstinspires.ftc.teamcode.vision.VisionStateManager;
import org.firstinspires.ftc.teamcode.vision.localization.HiveAngleEstimator;
import org.firstinspires.ftc.teamcode.vision.localization.LimelightLocalizer;
import org.firstinspires.ftc.teamcode.vision.tracking.*;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.List;

/** Sole camera owner. Polling is handled by the SDK; fusion happens after drivetrain odometry. */
public class VisionSubsystem extends SubsystemBase implements AutoCloseable {
    private final Limelight3A limelight;
    private final VisionStateManager stateManager;
    private final GameElementMap map = new GameElementMap();
    private final MultipleTelemetry telemetry;
    private final FtcDashboard dashboard;
    private LimelightLocalizer localizer;
    private LimelightLocalizer.Frame latestFrame;
    private Boolean configuredAlliance;
    private int localizationPipeline = -1;
    private double lastSnapshotTimestamp = Double.NaN;
    private String inputStatus = "Waiting for alliance";
    private boolean closed;

    public VisionSubsystem(HardwareMap hm, MultipleTelemetry telemetry, FtcDashboard dashboard) {
        this.telemetry = telemetry;
        this.dashboard = dashboard;
        limelight = hm.tryGet(Limelight3A.class, "limelight");
        stateManager = limelight == null ? null : new VisionStateManager(limelight);
        if (limelight != null) {
            limelight.setPollRateHz(50);
            limelight.start();
        } else inputStatus = "No Limelight: odometry only";
    }

    public boolean setPipeline(int pipeline) {
        if (stateManager == null || !stateManager.setCurrentPipeline(pipeline)) {
            inputStatus = "Pipeline request failed";
            latestFrame = null;
            return false;
        }
        latestFrame = null;
        if (localizer != null) localizer.invalidateBefore(LocalizationClock.now());
        return true;
    }

    /** Pure/cached processing only: called by DrivetrainLocalizer after its odometry update. */
    public void updateLocalization(double now) {
        if (localizer == null || stateManager == null || closed) return;
        if (!configuredAlliance.equals(RobotState.getInstance().isBlueTeam())) return;
        if (stateManager.getCurrentPipeline() == localizationPipeline) {
            localizer.update(latestFrame, now, RobotState.getInstance());
        }
    }

    public LimelightLocalizer getLocalizer() { return localizer; }

    @Override public void periodic() {
        if (limelight == null || closed) {
            telemetry.addData("Hive vision", inputStatus);
            return;
        }
        Boolean alliance = RobotState.getInstance().isBlueTeam();
        if (alliance == null) {
            latestFrame = null;
            telemetry.addData("Hive vision", "Waiting for alliance; odometry only");
            return;
        }
        if (!alliance.equals(configuredAlliance)) {
            HiveLocalizationConfig.Parameters config = new HiveLocalizationConfig.Parameters(alliance);
            configuredAlliance = alliance;
            localizer = null;
            localizationPipeline = config.pipeline;
            // Attempt once per alliance selection, rather than blocking on network retries each loop.
            if (!setPipeline(config.pipeline)) return;
            localizer = new LimelightLocalizer(config, LocalizationClock.now());
        }

        try {
            LLResult result = limelight.getLatestResult();
            if (stateManager.getCurrentPipeline() == GAME_ELEMENT_TRACKER_PIPELINE) {
                latestFrame = null;
                updateGameElements(result);
            } else {
                snapshot(result);
            }
        } catch (JSONException | RuntimeException e) {
            latestFrame = null;
            inputStatus = "Rejected camera input: " + e.getClass().getSimpleName();
        }
        telemetry.addData("Hive input", inputStatus);
        if (localizer != null) {
            telemetry.addData("Hive vision", localizer.getStatus());
            telemetry.addData("Hive accepted frames", localizer.getAcceptedCount());
            telemetry.addData("Hive last accepted capture (s)", localizer.getLastAcceptedTime());
            telemetry.addData("Hive last accepted pose (m)", localizer.getLastAcceptedPose());
            telemetry.addData("Hive last accepted age (s)",
                    LocalizationClock.now() - localizer.getLastAcceptedTime());
            HiveAngleEstimator.Result angle = localizer.getAngleResult();
            if (angle != null) {
                telemetry.addData("Hive angle status", angle.failure);
                if (angle.isValid()) {
                    telemetry.addData("Hive tilt (deg)", Math.toDegrees(angle.hiveAngle));
                    telemetry.addData("Hive level residual (deg)", Math.toDegrees(angle.orientationError));
                }
            }
        }
    }

    private void snapshot(LLResult result) throws JSONException {
        if (result == null) { latestFrame = null; inputStatus = "No result"; return; }
        JSONObject json = new JSONObject(result.toString());
        double timestamp = json.optDouble("ts", Double.NaN);
        if (timestamp == lastSnapshotTimestamp) return;
        lastSnapshotTimestamp = timestamp;
        JSONArray pose = json.optJSONArray("botpose");
        Pose3d raw = null;
        if (pose != null && pose.length() >= 6) {
            double[] v = new double[6];
            for (int i = 0; i < 6; i++) v[i] = pose.optDouble(i, Double.NaN);
            if (HiveAngleEstimator.finite(v)) {
                raw = new Pose3d(v[0], v[1], v[2], new Rotation3d(
                        Math.toRadians(v[3]), Math.toRadians(v[4]), Math.toRadians(v[5])));
            }
        }
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        int[] ids = new int[tags == null ? 0 : tags.size()];
        for (int i = 0; i < ids.length; i++) ids[i] = tags.get(i).getFiducialId();
        latestFrame = new LimelightLocalizer.Frame(timestamp, LocalizationClock.now(),
                System.currentTimeMillis(), result.getControlHubTimeStamp(),
                json.optDouble("cl", Double.NaN), json.optDouble("tl", Double.NaN),
                json.optInt("pID", -1), result.isValid(), raw,
                json.optInt("botpose_tagcount", 0), json.optDouble("botpose_avgdist", Double.NaN), ids);
        inputStatus = "MT1 snapshot";
    }

    private void updateGameElements(LLResult result) {
        RobotState rs = RobotState.getInstance();
        // TODO: Supply measured CameraPose for game-element projection; never inject fake odometry.
        if (rs.getEstimatedPose() == null || rs.getCameraPose() == null) return;
        limelight.updatePythonInputs(new LimelightPythonInput(
                rs.getEstimatedPose(), rs.getCameraPose()).toArray());
        if (result == null || !result.isValid()
                || result.getPipelineIndex() != GAME_ELEMENT_TRACKER_PIPELINE) return;
        double[] output = LimelightOutputReader.getFullPythonOutput(result, telemetry);
        if (output == null) return;
        double time = TimeTracker.convertTime(result.getControlHubTimeStamp() / 1000.0);
        for (int i = 0; i + VALUES_PER_ELEMENT <= output.length; i += VALUES_PER_ELEMENT) {
            int id = (int) output[i + ID_OFFSET];
            GameElementType type = calcElementType(id);
            if (id == EMPTY_ID || type == GameElementType.UNKNOWN) continue;
            map.addElement(new TrackedGameElement(id, type,
                    new Pose2d(output[i + X_OFFSET], output[i + Y_OFFSET], new Rotation2d()),
                    output[i + CONFIDENCE_OFFSET], time));
        }
        TelemetryPacket packet = new TelemetryPacket();
        map.drawMap(packet);
        dashboard.sendTelemetryPacket(packet);
    }

    @Override public void close() {
        closed = true;
        latestFrame = null;
        if (limelight != null) limelight.stop();
    }
}
