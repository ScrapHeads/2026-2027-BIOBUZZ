package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.VisionConstants.APRIL_TAG_PIPELINE;
import static org.firstinspires.ftc.teamcode.config.VisionConstants.GAME_ELEMENT_TRACKER_PIPELINE;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.VecBuilder;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.util.TimeTracker;
import org.firstinspires.ftc.teamcode.vision.VisionStateManager;
import org.firstinspires.ftc.teamcode.vision.tracking.GameElementMap;
import org.firstinspires.ftc.teamcode.vision.tracking.GameElementType;
import org.firstinspires.ftc.teamcode.vision.tracking.LimelightPythonInput;
import org.firstinspires.ftc.teamcode.vision.tracking.TrackedGameElement;

public class VisionSubsystem implements Subsystem {
    // Owns the Limelight and reads getPythonOutput()

    private final Limelight3A limelight;

    private final VisionStateManager stateManager;

    private final GameElementMap map;

    private final MultipleTelemetry telemetry;


    public VisionSubsystem (HardwareMap hm, MultipleTelemetry telemetry) {
        this.limelight = hm.get(Limelight3A.class, "limelight");
        this.telemetry = telemetry;
        this.stateManager = new VisionStateManager(limelight);
        this.map = new GameElementMap();
    }

    //TODO
    public Pose2d calcCameraPose () {
        return new Pose2d();
    }

    @Override
    public void periodic () {
//        limelight.updateRobotOrientation(robotRot);

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
            LimelightPythonInput input = new LimelightPythonInput(rs.getEstimatedPose(), rs.getCameraPose());
            limelight.updatePythonInputs(input.toArray());

            LLResult limelightResult = limelight.getLatestResult();
            //Python output
            double[] pyo = limelightResult.getPythonOutput();

            double limelightTime = TimeTracker.convertTime(limelightResult.getControlHubTimeStamp() / 1000.0);

            for (int ei = 0; ei < pyo.length - 4; ei += 4) {
                double id = pyo[ei];
                GameElementType type = calcElementType(id);
                Pose2d estimatedPose = new Pose2d(pyo[ei + 1], pyo[ei + 2], new Rotation2d());

                if (type == GameElementType.UNKNOWN) {
                    continue;
                }
                TrackedGameElement element = new TrackedGameElement(
                        id,
                        type,
                        estimatedPose,
                        pyo[ei + 3],
                        limelightTime
                );
            }
        }
    }

    private GameElementType calcElementType (double id) {
        switch (Math.floorDiv((int) id, 1000)) {
            case 0:
                return GameElementType.POLLEN;
            case 1:
                return GameElementType.BLUE_NECTAR;
            case 2:
                return GameElementType.RED_NECTAR;
            default:
                return GameElementType.UNKNOWN;
        }
    }

}
