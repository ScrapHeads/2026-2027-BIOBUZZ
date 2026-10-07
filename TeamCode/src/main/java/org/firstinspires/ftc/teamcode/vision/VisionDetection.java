package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.vision.tracking.GameElementType;

public class VisionDetection {

    /**
     * Raw detection from Python:
     * - type
     * - robot-relative X/Y
     * - box information
     * - area/fullness
     * - image location
     */

    private final GameElementType type;
    private final double forwardCm;
    private final double leftCm;

    public VisionDetection(
            GameElementType type,
            double forwardCm,
            double leftCm
    ) {
        this.type = type;
        this.forwardCm = forwardCm;
        this.leftCm = leftCm;
    }

    public GameElementType getType() {
        return type;
    }

    public double getForwardCm() {
        return forwardCm;
    }

    public double getLeftCm() {
        return leftCm;
    }

    public double distanceCm() {
        return Math.hypot(forwardCm, leftCm);
    }
}
