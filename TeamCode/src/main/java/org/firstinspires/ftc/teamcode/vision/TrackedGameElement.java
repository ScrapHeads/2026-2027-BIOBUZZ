package org.firstinspires.ftc.teamcode.vision;

public class TrackedGameElement {

    private final GameElementType type;

    private double fieldX;
    private double fieldY;

    private long lastSeenTime;

    public TrackedGameElement(
            GameElementType type,
            double fieldX,
            double fieldY
    ) {
        this.type = type;
        this.fieldX = fieldX;
        this.fieldY = fieldY;
        this.lastSeenTime = System.currentTimeMillis();
    }

    public double distanceTo(double x, double y) {
        return Math.hypot(
                fieldX - x,
                fieldY - y
        );
    }

    public void updatePosition(double x, double y) {
        fieldX = x;
        fieldY = y;
        lastSeenTime = System.currentTimeMillis();
    }

    public GameElementType getType() {
        return type;
    }

    public double getFieldX() {
        return fieldX;
    }

    public double getFieldY() {
        return fieldY;
    }
}