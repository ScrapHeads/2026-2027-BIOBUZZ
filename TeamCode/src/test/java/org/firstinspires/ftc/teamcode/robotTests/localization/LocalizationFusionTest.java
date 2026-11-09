package org.firstinspires.ftc.teamcode.robotTests.localization;

import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.*;
import org.firstinspires.ftc.teamcode.RilLib.Math.VecBuilder;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.config.HiveLocalizationConfig.Parameters;
import org.firstinspires.ftc.teamcode.vision.localization.LimelightLocalizer;
import org.firstinspires.ftc.teamcode.vision.localization.LimelightLocalizer.Frame;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class LocalizationFusionTest {
    private final RobotState state = RobotState.getInstance();
    @BeforeEach void reset() {
        state.resetPose(new Pose2d(1, 0, new Rotation2d()), 9);
        state.addOdometryObservation(new Pose2d(1, 0, new Rotation2d()), 10);
    }
    private Frame frame(double ts, int... ids) {
        return new Frame(ts, 10 + (ts - 100) * 0.01,
                10000 + (long) ((ts - 100) * 10), 10000 + (long) ((ts - 100) * 10),
                10, 20, 0, true,
                new Pose3d(1.1, 0, 0, new Rotation3d()), ids.length, 1, ids);
    }
    @Test void fusesCorrectedObservationOnceAndRejectsMixedHives() {
        LimelightLocalizer localizer = new LimelightLocalizer(new Parameters(false), 9);
        localizer.update(frame(100, 30, 31), 10, state);
        assertEquals(LimelightLocalizer.Status.ACCEPTED, localizer.getStatus());
        double fusedX = state.getEstimatedPose().getX();
        assertTrue(fusedX > 1 && fusedX < 1.1);
        localizer.update(frame(100, 30, 31), 10.01, state);
        assertEquals(LimelightLocalizer.Status.DUPLICATE, localizer.getStatus());
        assertEquals(fusedX, state.getEstimatedPose().getX());
        localizer.update(frame(101, 30, 38), 10.01, state);
        assertEquals(LimelightLocalizer.Status.WRONG_HIVE, localizer.getStatus());
        assertEquals(1, localizer.getAcceptedCount());
    }
    @Test void failedCorrectionNeverFallsBackToRawPose() {
        LimelightLocalizer localizer = new LimelightLocalizer(new Parameters(false), 9);
        Frame bad = new Frame(100, 10, 10000, 10000, 10, 20, 0, true,
                new Pose3d(1, 0, 0, new Rotation3d(0.5, 0, 0)), 1, 1, new int[]{30});
        localizer.update(bad, 10, state);
        assertEquals(LimelightLocalizer.Status.ANGLE_REJECTED, localizer.getStatus());
        assertEquals(1, state.getEstimatedPose().getX());
    }
    @Test void samplesCaptureTimeAndRejectsUnavailableHistoryAndFuture() {
        state.resetPose(new Pose2d(), 10);
        state.addOdometryObservation(new Pose2d(1, 0, new Rotation2d()), 11);
        assertEquals(0.5, state.sampleEstimatedPose(10.5).get().getX(), 1e-9);
        assertFalse(state.sampleEstimatedPose(9.9).isPresent());
        assertFalse(state.sampleEstimatedPose(11.1).isPresent());
        assertTrue(state.tryAddVisionObservation(new Pose2d(0.7, 0, new Rotation2d()),
                10.5, VecBuilder.fill(0.5, 0.5, 0.5)));
        assertTrue(state.getEstimatedPose().getX() > 1);
        assertFalse(state.tryAddVisionObservation(new Pose2d(), 10.5,
                VecBuilder.fill(0.5, 0.5, 0.5)));
        state.addOdometryObservation(new Pose2d(2, 0, new Rotation2d()), 11.6);
        assertFalse(state.sampleEstimatedPose(10.5).isPresent());
    }
    @Test void rejectsStaleCachedFramesAndCaptureTimeOutliers() {
        LimelightLocalizer stale = new LimelightLocalizer(new Parameters(false), 9);
        stale.update(frame(100, 30), 10.5, state);
        assertEquals(LimelightLocalizer.Status.STALE, stale.getStatus());
        state.resetPose(new Pose2d(-1, 0, new Rotation2d()), 9);
        state.addOdometryObservation(new Pose2d(-1, 0, new Rotation2d()), 10);
        LimelightLocalizer outlier = new LimelightLocalizer(new Parameters(false), 9);
        outlier.update(frame(100, 30), 10, state);
        assertEquals(LimelightLocalizer.Status.OUTLIER, outlier.getStatus());
        assertEquals(-1, state.getEstimatedPose().getX());
    }

    @Test void transformsCorrectedMapPoseIntoExistingFieldFrame() {
        double savedX = org.firstinspires.ftc.teamcode.config.HiveLocalizationConfig.RED_FIELD_X_M;
        double savedY = org.firstinspires.ftc.teamcode.config.HiveLocalizationConfig.RED_FIELD_Y_M;
        double savedYaw = org.firstinspires.ftc.teamcode.config.HiveLocalizationConfig.RED_FIELD_YAW_RAD;
        try {
            org.firstinspires.ftc.teamcode.config.HiveLocalizationConfig.RED_FIELD_X_M = 0.2;
            org.firstinspires.ftc.teamcode.config.HiveLocalizationConfig.RED_FIELD_Y_M = -0.1;
            org.firstinspires.ftc.teamcode.config.HiveLocalizationConfig.RED_FIELD_YAW_RAD = 0.6;
            Pose2d expected = new Pose2d(0.2 + 1.1 * Math.cos(0.6),
                    -0.1 + 1.1 * Math.sin(0.6), new Rotation2d(0.6));
            state.resetPose(expected, 9);
            state.addOdometryObservation(expected, 10);
            LimelightLocalizer localizer = new LimelightLocalizer(new Parameters(false), 9);
            localizer.update(frame(100, 30), 10, state);
            assertEquals(LimelightLocalizer.Status.ACCEPTED, localizer.getStatus());
            assertEquals(expected.getX(), localizer.getLastAcceptedPose().getX(), 1e-9);
            assertEquals(expected.getY(), localizer.getLastAcceptedPose().getY(), 1e-9);
            assertEquals(0.6, localizer.getLastAcceptedPose().getRotation().getRadians(), 1e-9);
        } finally {
            org.firstinspires.ftc.teamcode.config.HiveLocalizationConfig.RED_FIELD_X_M = savedX;
            org.firstinspires.ftc.teamcode.config.HiveLocalizationConfig.RED_FIELD_Y_M = savedY;
            org.firstinspires.ftc.teamcode.config.HiveLocalizationConfig.RED_FIELD_YAW_RAD = savedYaw;
        }
    }

    @Test void resetClearsVisionCorrectionsAndOldFrames() {
        state.tryAddVisionObservation(new Pose2d(1.2, 0, new Rotation2d()),
                9.9, VecBuilder.fill(0.5, 0.5, 0.5));
        long generation = state.getPoseGeneration();
        state.resetPose(new Pose2d(2, 1, new Rotation2d(0.4)), 11);
        assertTrue(state.getPoseGeneration() > generation);
        assertEquals(2, state.getEstimatedPose().getX());
        assertFalse(state.sampleEstimatedPose(10).isPresent());
        state.addOdometryObservation(new Pose2d(2.1, 1, new Rotation2d(0.4)), 11.1);
        assertEquals(2.1, state.getEstimatedPose().getX(), 1e-9);
    }
}
