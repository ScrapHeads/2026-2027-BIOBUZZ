package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.VisionConstants.*;
import static org.firstinspires.ftc.teamcode.vision.tracking.TrackedGameElement.calcElementType;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.MathUtil;
import org.firstinspires.ftc.teamcode.RilLib.Math.VecBuilder;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.util.ConversionUtil;
import org.firstinspires.ftc.teamcode.vision.CameraPose;
import org.firstinspires.ftc.teamcode.vision.tracking.LimelightOutputReader;
import org.firstinspires.ftc.teamcode.util.TimeTracker;
import org.firstinspires.ftc.teamcode.vision.VisionStateManager;
import org.firstinspires.ftc.teamcode.vision.tracking.GameElementMap;
import org.firstinspires.ftc.teamcode.vision.tracking.GameElementType;
import org.firstinspires.ftc.teamcode.vision.tracking.LimelightPythonInput;
import org.firstinspires.ftc.teamcode.vision.tracking.TrackedGameElement;
import org.json.JSONArray;
import org.json.JSONObject;

import java.lang.reflect.Field;

public class VisionSubsystem implements Subsystem {
    // Owns the Limelight and reads getPythonOutput()

    private final Limelight3A limelight;

    private final VisionStateManager stateManager;

    private final GameElementMap map;

    private final MultipleTelemetry telemetry;

    private final FtcDashboard dashboard;


    public VisionSubsystem (HardwareMap hm, MultipleTelemetry telemetry, FtcDashboard dashboard) {
        this.limelight = hm.get(Limelight3A.class, "limelight");
        this.telemetry = telemetry;
        this.dashboard= dashboard;
        this.stateManager = new VisionStateManager(limelight);
        this.map = new GameElementMap();

        limelight.start();
    }

    //TODO
    public Pose2d calcCameraPose () {
        return new Pose2d();
    }

    public void setPipeline (int pipeline) {
        stateManager.setCurrentPipeline(pipeline);
    }

    @Override
    public void periodic () {
//        limelight.updateRobotOrientation(robotRot);

//        telemetry.addData("Pipeline", limelight.getStatus().toString());
        telemetry.addLine();

        if (stateManager.getCurrentPipeline() == APRIL_TAG_PIPELINE) {
            //TODO calculate the real robot orientation with the change in limelight yaw
            limelight.updateRobotOrientation(0.0);

            LLResult limeLightResult = limelight.getLatestResult();
            Pose3D camPose3D = limeLightResult.getBotpose_MT2();

            Pose2d poseRobot = new Pose2d(
                    camPose3D.getPosition().x,
                    camPose3D.getPosition().y,
                    new Rotation2d(camPose3D.getOrientation().getYaw(AngleUnit.RADIANS))
            );

            double limelightTime = TimeTracker.convertTime(limeLightResult.getControlHubTimeStamp() / 1000.0);

            if (poseRobot.getX() == 0 && poseRobot.getY() == 0) return;

            RobotState.getInstance().addVisionObservation(poseRobot, limelightTime,
                    VecBuilder.fill(
                            Math.pow(0.8, limeLightResult.getFiducialResults().size()) * (limeLightResult.getBotposeAvgDist()) * 2,
                            Math.pow(0.8, limeLightResult.getFiducialResults().size()) * (limeLightResult.getBotposeAvgDist()) * 2,
                            9999999));

//            Pose2d tagLocation = RobotState.getInstance().isBlueTeam() ? blueTagPose : redTagPose;
//            double distance = 100 * RobotState.getInstance().getEstimatedPose().getTranslation().getDistance(tagLocation.getTranslation());
        } else if (stateManager.getCurrentPipeline() == GAME_ELEMENT_TRACKER_PIPELINE) {
            RobotState rs = RobotState.getInstance();

            // Fillers to test the system set these to what pose is needed for testing
            // They should both be in meters and degrees
            rs.addOdometryObservation(new Pose2d(0, 0, new Rotation2d(.785398)), TimeTracker.getTime());
            rs.setCameraPose(new CameraPose(0, 0, .040, 60, 0));

            LimelightPythonInput input = new LimelightPythonInput(rs.getEstimatedPose(), rs.getCameraPose());
            limelight.updatePythonInputs(input.toArray());

            LLResult limelightResult = limelight.getLatestResult();
            //Python output

            double[] pyo = null;

            if (limelightResult != null) {
                pyo = LimelightOutputReader.getFullPythonOutput(limelightResult, telemetry);
//                pyo = limelightResult.getPythonOutput();
                telemetry.addData("Received Values", pyo.length);
            }

            if (pyo != null && pyo.length >= VALUES_PER_ELEMENT) {
                double limelightTime = TimeTracker.convertTime(
                        limelightResult.getControlHubTimeStamp() / 1000.0
                );

                for (int ei = 0; ei + VALUES_PER_ELEMENT <= pyo.length; ei += VALUES_PER_ELEMENT) {
                    int id = (int) pyo[ei + ID_OFFSET];

                    // Skip unused slots
                    if (id == EMPTY_ID) {
                        continue;
                    }

                    GameElementType type = calcElementType(id);

                    if (type == GameElementType.UNKNOWN) {
                        continue;
                    }

                    Pose2d estimatedPose = new Pose2d(
                            pyo[ei + X_OFFSET],
                            pyo[ei + Y_OFFSET],
                            new Rotation2d()
                    );

                    double confidence = pyo[ei + CONFIDENCE_OFFSET];

                    TrackedGameElement element = new TrackedGameElement(
                            id,
                            type,
                            estimatedPose,
                            confidence,
                            limelightTime
                    );

                    map.addElement(element);

                    // Add or update the element in your persistent map
                    // elementMapEstimator.addOrUpdate(element);
                }

                telemetry.addData("Elements id", map.getSize());
                telemetry.addData("Element Amounts", map.returnAmountOfEachElement());
            }
            TelemetryPacket packet = new TelemetryPacket();
            map.drawMap(packet);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
