package org.firstinspires.ftc.teamcode.vision.localization;

import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose3d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.VecBuilder;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.config.HiveLocalizationConfig;
import org.firstinspires.ftc.teamcode.config.HiveLocalizationConfig.Parameters;

import java.util.Optional;

/** Upstream MT1 correction and gates; RobotState remains the only pose estimator. */
public final class LimelightLocalizer {
    public enum Status { NO_FRAME, DISABLED, CONFIGURATION, DUPLICATE, CLOCK_RESTART, STALE,
        INVALID_FRAME, WRONG_PIPELINE, WRONG_HIVE, ANGLE_REJECTED, NO_HISTORY, OUTLIER, ACCEPTED }

    /** Immutable snapshot; camera time is an identity token, not a local timestamp. */
    public static final class Frame {
        public final double cameraTime, readTime, captureMs, processingMs, averageDistance;
        public final long readWallMs, receiptWallMs;
        public final int pipeline, tagCount;
        public final boolean valid;
        public final Pose3d rawPose;
        private final int[] tagIds;
        public Frame(double cameraTime, double readTime, long readWallMs, long receiptWallMs,
                     double captureMs, double processingMs, int pipeline, boolean valid,
                     Pose3d rawPose, int tagCount, double averageDistance, int[] tagIds) {
            this.cameraTime = cameraTime; this.readTime = readTime;
            this.readWallMs = readWallMs; this.receiptWallMs = receiptWallMs;
            this.captureMs = captureMs; this.processingMs = processingMs;
            this.pipeline = pipeline; this.valid = valid; this.rawPose = rawPose;
            this.tagCount = tagCount; this.averageDistance = averageDistance;
            this.tagIds = tagIds == null ? new int[0] : tagIds.clone();
        }
    }

    private final Parameters config;
    private final HiveAngleEstimator solver = new HiveAngleEstimator();
    private final VisionFrameClock clock = new VisionFrameClock();
    private Status status = Status.NO_FRAME;
    private HiveAngleEstimator.Result angleResult;
    private Pose2d correctedPose;
    private double lastAcceptedTime = Double.NaN;
    private Pose2d lastAcceptedPose;
    private double minimumCaptureTime;
    private long acceptedCount;
    public LimelightLocalizer(Parameters config, double startTime) {
        this.config = config;
        minimumCaptureTime = startTime;
    }
    public void invalidateBefore(double time) { minimumCaptureTime = time; }
    public Status getStatus() { return status; }
    public HiveAngleEstimator.Result getAngleResult() { return angleResult; }
    public Pose2d getCorrectedPose() { return correctedPose; }
    public double getLastAcceptedTime() { return lastAcceptedTime; }
    public Pose2d getLastAcceptedPose() { return lastAcceptedPose; }
    public long getAcceptedCount() { return acceptedCount; }

    public void update(Frame frame, double now, RobotState state) {
        angleResult = null;
        correctedPose = null;
        if (!HiveLocalizationConfig.ENABLED) { status = Status.DISABLED; return; }
        if (!validConfig()) { status = Status.CONFIGURATION; return; }
        if (frame == null) { status = Status.NO_FRAME; return; }
        if (!Double.isFinite(now) || !Double.isFinite(frame.readTime)
                || frame.readTime > now || now - frame.readTime > config.maxAge) {
            status = Status.STALE; return;
        }
        VisionFrameClock.Reading timing = clock.read(frame.cameraTime, frame.readTime,
                frame.readWallMs, frame.receiptWallMs, frame.captureMs, frame.processingMs,
                config.transportDelay, config.maxAge);
        switch (timing.status) {
            case DUPLICATE: status = Status.DUPLICATE; return;
            case RESTART: status = Status.CLOCK_RESTART; return;
            case STALE: status = Status.STALE; return;
            case INVALID: status = Status.INVALID_FRAME; return;
            default: break;
        }
        double captureTime = timing.captureTime;
        if (!Double.isFinite(now) || captureTime < minimumCaptureTime || captureTime > now
                || now - captureTime > config.maxAge) { status = Status.STALE; return; }
        if (frame.pipeline != config.pipeline) { status = Status.WRONG_PIPELINE; return; }
        if (!frame.valid || frame.rawPose == null || frame.tagCount < config.minTags
                || frame.tagIds.length < frame.tagCount
                || !Double.isFinite(frame.averageDistance) || frame.averageDistance <= 0
                || frame.averageDistance > config.maxDistance) {
            status = Status.INVALID_FRAME; return;
        }
        // Conservative: the SDK detection list does not identify the exact MT1 contributors.
        for (int id : frame.tagIds) {
            if (id < config.firstTag || id > config.lastTag) {
                status = Status.WRONG_HIVE; return;
            }
        }
        angleResult = solver.estimate(frame.rawPose, config.geometry, config.maxTilt,
                config.maxLevelError, config.minSensitivity, config.rootMargin);
        if (!angleResult.isValid()) { status = Status.ANGLE_REJECTED; return; }
        Pose3d mapPose = angleResult.correctedPose;
        double c = Math.cos(config.fieldYaw), s = Math.sin(config.fieldYaw);
        correctedPose = new Pose2d(config.fieldX + c * mapPose.getX() - s * mapPose.getY(),
                config.fieldY + s * mapPose.getX() + c * mapPose.getY(),
                new Rotation2d(mapPose.getRotation().getZ() + config.fieldYaw));
        if (!RobotState.validPose(correctedPose) || correctedPose.getX() < config.minX
                || correctedPose.getX() > config.maxX || correctedPose.getY() < config.minY
                || correctedPose.getY() > config.maxY) { status = Status.OUTLIER; return; }
        Optional<Pose2d> prediction = state.sampleEstimatedPose(captureTime);
        if (!prediction.isPresent()) { status = Status.NO_HISTORY; return; }
        Pose2d expected = prediction.get();
        if (Math.hypot(correctedPose.getX() - expected.getX(),
                correctedPose.getY() - expected.getY()) > config.maxTranslationError
                || Math.abs(HiveAngleEstimator.wrap(correctedPose.getRotation().getRadians()
                - expected.getRotation().getRadians())) > config.maxHeadingError) {
            status = Status.OUTLIER; return;
        }
        // Bounded heuristic, not a covariance estimate from the SDK.
        double quality = (1 + 0.15 * frame.averageDistance * frame.averageDistance)
                / Math.sqrt(frame.tagCount);
        quality *= 1 + angleResult.orientationError / config.maxLevelError
                + Math.min(3, 0.2 / angleResult.sensitivity);
        double xyStd = Math.min(config.maxPositionStd, config.positionStd * Math.max(1, quality));
        double yawStd = Math.min(config.maxHeadingStd, config.headingStd * Math.max(1, quality));
        if (state.tryAddVisionObservation(correctedPose, captureTime,
                VecBuilder.fill(xyStd, xyStd, yawStd))) {
            status = Status.ACCEPTED;
            lastAcceptedTime = captureTime;
            lastAcceptedPose = correctedPose;
            acceptedCount++;
        } else status = Status.NO_HISTORY;
    }

    private boolean validConfig() {
        return config != null && config.pipeline >= 0 && config.pipeline <= 9 && config.minTags > 0
                && HiveAngleEstimator.finite(config.fieldX, config.fieldY, config.fieldYaw,
                config.maxDistance, config.maxAge, config.maxTranslationError,
                config.maxHeadingError, config.minX, config.maxX, config.minY, config.maxY,
                config.transportDelay, config.positionStd, config.headingStd,
                config.maxPositionStd, config.maxHeadingStd)
                && config.maxDistance > 0 && config.maxAge > 0 && config.maxAge < 1.5
                && config.maxTranslationError > 0 && config.maxHeadingError > 0
                && config.minX < config.maxX && config.minY < config.maxY
                && config.transportDelay >= 0 && config.positionStd > 0 && config.headingStd > 0
                && config.maxPositionStd >= config.positionStd
                && config.maxHeadingStd >= config.headingStd;
    }
}
