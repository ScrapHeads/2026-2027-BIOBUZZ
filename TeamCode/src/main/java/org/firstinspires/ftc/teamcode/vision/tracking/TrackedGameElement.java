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

    private final GameElementType type;

    private final double id;

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

    public double getId () {
        return id;
    }

    public GameElementType getType() {
        return type;
    }

    public GameElementState getState () {
        return state;
    }

    public void setState (GameElementState state) {
        this.state = state;
    }

    public Pose2d getPose() {
        return estimatedPose;
    }
}