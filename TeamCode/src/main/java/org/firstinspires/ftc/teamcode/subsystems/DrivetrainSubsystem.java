package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.pedro.Constants.createFollower;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class DrivetrainSubsystem implements Subsystem {
    private final Follower follower;

    private final MultipleTelemetry multipleTelemetry;

    public DrivetrainSubsystem (HardwareMap hm, MultipleTelemetry multipleTelemetry) {
        this.multipleTelemetry = multipleTelemetry;

        follower = createFollower(hm);
    }

    public void drive(double forward, double strafe, double turn) {
        follower.manual(forward, strafe, turn);
    }

    public void driveFieldCentric(
            double forward,
            double strafe,
            double turn,
            double headingOffset) {

        double heading = follower.pose().heading() - headingOffset;

        DrivePowers powers = ManualDrive.fieldCentric(
                forward,
                strafe,
                turn,
                heading
        );

        follower.manual(powers);
    }

    // Return control to manual driving
    public void resumeManualDrive() {
        follower.manual(0, 0, 0);
    }

    public void followPath (Path path) {
        follower.follow(path);
    }

    public boolean isBusy() {
        return follower.isBusy();
    }

    public Pose getPose() {
        return follower.pose();
    }

    public void stop() {
        follower.stop();
    }

    @Override
    public void periodic() {
        follower.update();
    }
}
