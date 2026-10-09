package org.firstinspires.ftc.teamcode.vision;

public class CameraPose {
    public double cameraX;
    public double cameraY;
    public double cameraHeight;
    public double cameraPitch;
    public double cameraYaw;

    public CameraPose(double cameraX, double cameraY, double cameraHeight, double cameraPitch, double cameraYaw) {
        this.cameraX = cameraX;
        this.cameraY = cameraY;
        this.cameraHeight = cameraHeight;
        this.cameraPitch = cameraPitch;
        this.cameraYaw = cameraYaw;
    }

    public double getCameraX() {
        return cameraX;
    }

    public double getCameraY() {
        return cameraY;
    }

    public double getCameraHeight() {
        return cameraHeight;
    }

    public double getCameraPitch() {
        return cameraPitch;
    }

    public double getCameraYaw() {
        return cameraYaw;
    }

    public void setCameraX(double cameraX) {
        this.cameraX = cameraX;
    }

    public void setCameraY(double cameraY) {
        this.cameraY = cameraY;
    }

    public void setCameraHeight(double cameraHeight) {
        this.cameraHeight = cameraHeight;
    }

    public void setCameraPitch(double cameraPitch) {
        this.cameraPitch = cameraPitch;
    }

    public void setCameraYaw(double cameraYaw) {
        this.cameraYaw = cameraYaw;
    }
}
