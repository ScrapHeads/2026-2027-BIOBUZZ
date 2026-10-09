package org.firstinspires.ftc.teamcode.vision.tracking;

import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.vision.CameraPose;

public class LimelightPythonInput {
    // This assumes positive X is forward, positive Y is left, and positive rotation is counterclockwise in the robot coordinate system.
    public final double robotX;
    public final double robotY;
    public final double robotHeading;
    public final double cameraX;
    public final double cameraY;
    public final double cameraHeight;
    public final double cameraPitch;
    public final double cameraYaw;

    public LimelightPythonInput(Pose2d pose, CameraPose cameraPose) {
        this.robotX = pose.getX();
        this.robotY = pose.getY();
        this.robotHeading = pose.getRotation().getRadians();
        this.cameraX = cameraPose.getCameraX();
        this.cameraY = cameraPose.getCameraY();
        this.cameraHeight = cameraPose.getCameraHeight();
        this.cameraPitch = cameraPose.getCameraPitch();
        this.cameraYaw = cameraPose.getCameraYaw();
    }

    public double[] toArray() {
        return new double[] {
                robotX, robotY, robotHeading,
                cameraX, cameraY, cameraHeight,
                cameraPitch, cameraYaw
        };
    }
}
