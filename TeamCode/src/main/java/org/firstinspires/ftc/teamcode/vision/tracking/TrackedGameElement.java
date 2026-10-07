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

    private final int id;

    private Pose2d estimatedPose;

    private float confidence;

    private long lastSeenTime;

    private double observationCount;
    private GameElementState state;

    public TrackedGameElement(
            GameElementType type,
            int id,
            Pose2d estimatedPose,
            float confidence,
            long lastSeenTime,
            double observationCount,
            GameElementState state
    ) {
        this.type = type;
        this.id = id;
        this.estimatedPose = estimatedPose;
        this.confidence = confidence;
        this.lastSeenTime = lastSeenTime;
        this.observationCount = observationCount;
        this.state = state;
    }

    public void updatePosition(Pose2d newEstimatedPose, long lastSeenTime, float confidence) {
        this.estimatedPose = newEstimatedPose;
        this.lastSeenTime = lastSeenTime;
        this.confidence = confidence;
        observationCount++;
    }

    public int getId () {
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