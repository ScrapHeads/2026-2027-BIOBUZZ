package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.VisionConstants.APRIL_TAG_PIPELINE;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.VecBuilder;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.util.TimeTracker;
import org.firstinspires.ftc.teamcode.vision.VisionStateManager;

public class VisionSubsystem implements Subsystem {
    // Owns the Limelight and reads getPythonOutput()

    private final Limelight3A limelight;

    private final VisionStateManager stateManager;

    private final MultipleTelemetry telemetry;


    public VisionSubsystem (HardwareMap hm, MultipleTelemetry telemetry) {
        this.limelight = hm.get(Limelight3A.class, "limelight");
        this.telemetry = telemetry;
        this.stateManager = new VisionStateManager(limelight);
    }

    @Override
    public void periodic () {
//        limelight.updateRobotOrientation(robotRot);

        if (stateManager.getCurrentPipeline() == APRIL_TAG_PIPELINE) {
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
        }
    }

}
