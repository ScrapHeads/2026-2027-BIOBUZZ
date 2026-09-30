package org.firstinspires.ftc.teamcode.util;

public class PIDFValues {
    public double kP;
    public double kI;
    public double kD;
    public double kF;

    public double lastP;
    public double lastI;
    public double lastD;
    public double lastF;

    public PIDFValues(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;

        setLasts();
    }

    public boolean hasChanged() {
        return kP != lastP ||
                kI != lastI ||
                kD != lastD ||
                kF != lastF;
    }

    public void setLasts() {
        lastP = kP;
        lastI = kI;
        lastD = kD;
        lastF = kF;
    }

    @Override
    public String toString() {
        return String.format(
                "P: %.4f, I: %.4f, D: %.4f, F: %.4f",
                kP, kI, kD, kF
        );
    }
}
