package org.firstinspires.ftc.teamcode.robotTests;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import org.firstinspires.ftc.teamcode.RilLib.Math.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Numbers.N3;
import org.firstinspires.ftc.teamcode.RilLib.Math.VecBuilder;
import org.firstinspires.ftc.teamcode.RilLib.Math.Vector;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.util.TimeTracker;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;


public class RobotStateTest {

    public Vector<N3> createDummyVector() {
        double fakeLimelightInputX = .5;
        double fakeLimeLightInputY = .5;
        double headingConfidence = 9999999;

        return VecBuilder.fill(
                fakeLimelightInputX,
                fakeLimeLightInputY,
                headingConfidence
        );
    }

    @Test
    @DisplayName("RobotState setAll test")
    public void robotStateSetAllTest () {
        Pose2d pose2d = new Pose2d(1, 1, new Rotation2d(0));
        boolean isBlue = true;
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(1, 1, .25);
        RobotState robotState = RobotState.getInstance();
        robotState.setAll(pose2d, isBlue, chassisSpeeds);

        assertEquals(pose2d, robotState.getOdometryPose());
        assertEquals(pose2d, robotState.getEstimatedPose());
        assertEquals(isBlue, robotState.isBlueTeam());
        assertEquals(chassisSpeeds, robotState.getChassisSpeeds());
    }


    @Test
    @DisplayName("addVisionObservationTest")
    public void addVisionObservationTest() {
        RobotState robotState = RobotState.getInstance();
        TimeTracker.setOffset();

        // 1. Must add an initial odometry observation first to initialize poseEstimator and its buffer
        Pose2d initialOdoPose = new Pose2d(0, 0, new Rotation2d(0));
        robotState.addOdometryObservation(initialOdoPose, TimeTracker.getTime());

        // 2. Add vision observation to correct the pose estimate
        Pose2d visionPose2d = new Pose2d(1, 1, new Rotation2d(Math.toRadians(30)));
        Vector<N3> dummyVector = createDummyVector();
        robotState.addVisionObservation(visionPose2d, TimeTracker.getTime(), dummyVector);

        assertNotNull(robotState.getEstimatedPose(), "Estimated pose should not be null after vision observation");
        System.out.println("Fused Estimated Pose: " + robotState.getEstimatedPose());
        assertNotNull(robotState.getEstimatedPose());
    }

    @Test
    @DisplayName("addOdometryObservationTest")
    public void addOdometryObservationTest() {
        RobotState robotState = RobotState.getInstance();
        Pose2d odoPose2d = new Pose2d(1, 1, new Rotation2d(180));
        TimeTracker.setOffset();
        robotState.addOdometryObservation(odoPose2d, TimeTracker.getTime());
        assertEquals(odoPose2d, robotState.getOdometryPose());
    }

    @Test
    @DisplayName("get/set isBlueTeam test")
    public void getSetIsBlueTeamTest () {
        RobotState robotState = RobotState.getInstance();
        boolean isBlueTeam = false;
        robotState.setTeam(isBlueTeam);
        assertEquals(isBlueTeam, robotState.isBlueTeam());
    }

    @Test
    @DisplayName("get/set heading offset test")
    public void getSetHeadingOffsetTest () {
        RobotState robotState = RobotState.getInstance();
        Rotation2d headingOffset = new Rotation2d(80);
        robotState.setHeadingOffset(headingOffset);
        assertEquals(headingOffset, robotState.getHeadingOffset());
    }

    @Test
    @DisplayName("get/set chassis speeds test")
    public void getSetChassisSpeedsTest () {
        RobotState robotState = RobotState.getInstance();
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(1, 1, .25);
        robotState.setChassisSpeeds(chassisSpeeds);
        assertEquals(chassisSpeeds, robotState.getChassisSpeeds());
    }
}
