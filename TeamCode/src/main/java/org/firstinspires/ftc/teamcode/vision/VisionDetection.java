package org.firstinspires.ftc.teamcode.vision;

public class VisionDetection {

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
