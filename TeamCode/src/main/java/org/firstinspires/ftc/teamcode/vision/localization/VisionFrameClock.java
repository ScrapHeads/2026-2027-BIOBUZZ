package org.firstinspires.ftc.teamcode.vision.localization;

/** Physical-frame deduplication and approximate capture time in local monotonic seconds. */
public final class VisionFrameClock {
    public enum Status { NEW, DUPLICATE, RESTART, STALE, INVALID }
    public static final class Reading {
        public final Status status;
        public final double captureTime;
        private Reading(Status status, double time) { this.status = status; captureTime = time; }
    }
    private double lastCameraTime = Double.NaN;
    private double lastCaptureTime = Double.NEGATIVE_INFINITY;
    private double lastReadTime = Double.NaN;
    private long lastWallTime;

    public Reading read(double cameraTime, double readTime, long wallTime, long receiptWallTime,
                        double captureMs, double processingMs, double transportDelay, double maxAge) {
        if (!HiveAngleEstimator.finite(cameraTime, readTime, captureMs, processingMs,
                transportDelay, maxAge) || cameraTime <= 0 || captureMs < 0 || processingMs < 0
                || transportDelay < 0 || maxAge <= 0) return result(Status.INVALID);
        // Mark even invalid/stale physical frames consumed; never retry them with a newer time.
        if (cameraTime == lastCameraTime) return result(Status.DUPLICATE);
        boolean restart = Double.isFinite(lastCameraTime) && cameraTime < lastCameraTime;
        lastCameraTime = cameraTime;
        double wallDelta = Double.isFinite(lastReadTime)
                ? (wallTime - lastWallTime) * 0.001 - (readTime - lastReadTime) : 0;
        lastReadTime = readTime;
        lastWallTime = wallTime;
        if (restart || Math.abs(wallDelta) > 0.1) return result(Status.RESTART);
        double receiptAge = (wallTime - receiptWallTime) * 0.001;
        double age = receiptAge + (captureMs + processingMs) * 0.001 + transportDelay;
        if (receiptAge < 0 || age > maxAge) return result(Status.STALE);
        double captureTime = readTime - age;
        if (captureTime <= lastCaptureTime) return result(Status.STALE);
        lastCaptureTime = captureTime;
        return new Reading(Status.NEW, captureTime);
    }
    private static Reading result(Status status) { return new Reading(status, Double.NaN); }
}
