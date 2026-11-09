package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.RilLib.Math.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Matrix;
import org.firstinspires.ftc.teamcode.RilLib.Math.Numbers.N1;
import org.firstinspires.ftc.teamcode.RilLib.Math.Numbers.N3;
import org.firstinspires.ftc.teamcode.RilLib.Math.PoseEstimator;
import org.firstinspires.ftc.teamcode.util.LocalizationClock;
import org.firstinspires.ftc.teamcode.vision.CameraPose;

import java.util.NavigableSet;
import java.util.Optional;
import java.util.TreeSet;

/** Serializable robot pose snapshot. Position: meters; heading: radians. */
public class RobotState {
    private transient PoseEstimator poseEstimator;
    // Mirror actual sample bounds because PoseEstimator.sampleAt clamps its input.
    private final transient NavigableSet<Double> odometryTimes = new TreeSet<>();
    private transient double lastVisionTime = Double.NEGATIVE_INFINITY;
    private transient long poseGeneration;
    private transient Rotation2d headingOffset = new Rotation2d();
    private Pose2d odometryPose;
    private Pose2d estimatedPose;
    private CameraPose cameraPose;
    private ChassisSpeeds chassisSpeeds;
    private Boolean isBlue;
    private static RobotState instance;

    private RobotState() {}
    public static synchronized RobotState getInstance() {
        if (instance == null) instance = new RobotState();
        return instance;
    }

    /** Reset both odometry origin and fusion history. Drivetrain adopts this origin next update. */
    public synchronized void resetPose(Pose2d pose, double time) {
        if (!validPose(pose) || !Double.isFinite(time)) {
            throw new IllegalArgumentException("A finite pose and monotonic timestamp are required");
        }
        if (poseEstimator == null) poseEstimator = new PoseEstimator(pose);
        else poseEstimator.resetPose(pose);
        odometryTimes.clear();
        lastVisionTime = Double.NEGATIVE_INFINITY;
        odometryPose = pose;
        poseEstimator.updateWithTime(time, pose);
        odometryTimes.add(time);
        estimatedPose = poseEstimator.getEstimatedPosition();
        poseGeneration++;
    }

    public synchronized long getPoseGeneration() { return poseGeneration; }

    public synchronized void setAll(Pose2d pose, Boolean blue, ChassisSpeeds speeds) {
        resetPose(pose, LocalizationClock.now());
        isBlue = blue;
        chassisSpeeds = speeds;
    }

    /** Handoff uses the saved fused pose, not the old raw-odometry coordinate offset. */
    public synchronized void setAll(RobotState other) {
        if (other == null) return;
        Pose2d pose = other.getEstimatedPose();
        if (pose == null) pose = other.getOdometryPose();
        setAll(pose, other.isBlueTeam(), other.getChassisSpeeds());
        cameraPose = other.getCameraPose();
    }

    public synchronized void addOdometryObservation(Pose2d pose, double time) {
        if (!validPose(pose) || !Double.isFinite(time)) return;
        if (!odometryTimes.isEmpty() && time < odometryTimes.last()) return;
        if (poseEstimator == null) {
            resetPose(pose, time);
            return;
        }
        odometryPose = pose;
        poseEstimator.updateWithTime(time, pose);
        odometryTimes.add(time);
        while (!odometryTimes.isEmpty() && time - odometryTimes.first() >= 1.5) {
            odometryTimes.pollFirst();
        }
        estimatedPose = poseEstimator.getEstimatedPosition();
    }

    /** Strict sampling: unavailable capture times must not silently become endpoint samples. */
    public synchronized Optional<Pose2d> sampleEstimatedPose(double time) {
        if (!Double.isFinite(time) || poseEstimator == null || odometryTimes.isEmpty()
                || time < odometryTimes.first() || time > odometryTimes.last()) {
            return Optional.empty();
        }
        return poseEstimator.sampleAt(time);
    }

    public synchronized boolean tryAddVisionObservation(Pose2d pose, double time,
                                                         Matrix<N3, N1> stdDevs) {
        if (!validPose(pose) || stdDevs == null || time <= lastVisionTime
                || !sampleEstimatedPose(time).isPresent()) return false;
        for (int i = 0; i < 3; i++) {
            if (!Double.isFinite(stdDevs.get(i, 0)) || stdDevs.get(i, 0) <= 0) return false;
        }
        poseEstimator.addVisionMeasurement(pose, time, stdDevs);
        lastVisionTime = time;
        estimatedPose = poseEstimator.getEstimatedPosition();
        return true;
    }

    public void addVisionObservation(Pose2d pose, double time, Matrix<N3, N1> stdDevs) {
        tryAddVisionObservation(pose, time, stdDevs);
    }

    public static boolean validPose(Pose2d pose) {
        return pose != null && Double.isFinite(pose.getX()) && Double.isFinite(pose.getY())
                && Double.isFinite(pose.getRotation().getRadians());
    }
    public synchronized Boolean isBlueTeam() { return isBlue; }
    public synchronized void setTeam(Boolean blue) { isBlue = blue; }
    public synchronized Pose2d getOdometryPose() { return odometryPose; }
    public synchronized Pose2d getEstimatedPose() { return estimatedPose; }
    public synchronized void setCameraPose(CameraPose pose) { cameraPose = pose; }
    public synchronized CameraPose getCameraPose() { return cameraPose; }
    public synchronized void setHeadingOffset(Rotation2d offset) { headingOffset = offset; }
    public synchronized Rotation2d getHeadingOffset() { return headingOffset; }
    public synchronized void setChassisSpeeds(ChassisSpeeds speeds) { chassisSpeeds = speeds; }
    public synchronized ChassisSpeeds getChassisSpeeds() { return chassisSpeeds; }
    @Override public synchronized String toString() {
        return "RobotState{pose=" + odometryPose + ", estPose=" + estimatedPose
                + ", isBlue=" + isBlue + "}";
    }
}
