package org.firstinspires.ftc.teamcode.vision.tracking;

import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;

public class TrackedGameElement {

    /**
     * Persistent representation of one element:
     * - unique ID
     * - type
     * - estimated field position
     * - confidence
     * - last seen
     * - observation count
     * - tracking state
     */

    private final double id;

    private final GameElementType type;

    private Pose2d estimatedPose;

    private double confidence;

    private double lastSeenTime;

    private double observationCount;
    private GameElementState state;

    public TrackedGameElement(
            double id,
            GameElementType type,
            Pose2d estimatedPose,
            double confidence,
            double lastSeenTime
    ) {
        this.type = type;
        this.id = id;
        this.estimatedPose = estimatedPose;
        this.confidence = confidence;
        this.lastSeenTime = lastSeenTime;
        this.observationCount = 1;
        this.state = GameElementState.ACTIVE;
    }

    public void updatePosition(Pose2d newEstimatedPose, double lastSeenTime, float confidence) {
        this.estimatedPose = newEstimatedPose;
        this.lastSeenTime = lastSeenTime;
        this.confidence = confidence;
        observationCount++;
    }

    public double getId() {
        return id;
    }

    public GameElementType getType() {
        return type;
    }

    public Pose2d getEstimatedPose() {
        return estimatedPose;
    }

    public void setEstimatedPose(Pose2d estimatedPose) {
        this.estimatedPose = estimatedPose;
    }

    public double getConfidence() {
        return confidence;
    }

    public void setConfidence(double confidence) {
        this.confidence = confidence;
    }

    public double getLastSeenTime() {
        return lastSeenTime;
    }

    public void setLastSeenTime(double lastSeenTime) {
        this.lastSeenTime = lastSeenTime;
    }

    public double getObservationCount() {
        return observationCount;
    }

    public void setObservationCount(double observationCount) {
        this.observationCount = observationCount;
    }

    public GameElementState getState() {
        return state;
    }

    public void setState(GameElementState state) {
        this.state = state;
    }

    public static GameElementType calcElementType (double id) {
        switch (Math.floorDiv((int) id, 1000)) {
            case 1:
                return GameElementType.POLLEN;
            case 2:
                return GameElementType.BLUE_NECTAR;
            case 3:
                return GameElementType.RED_NECTAR;
            default:
                return GameElementType.UNKNOWN;
        }
    }
}