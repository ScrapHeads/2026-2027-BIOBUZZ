package org.firstinspires.ftc.teamcode.commands.drive;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.util.TimeTracker;

public class DriveContinuous extends CommandBase {

        private final DrivetrainSubsystem drivetrain;
        private final GamepadEx driver;
        private final MultipleTelemetry multipleTelemetry;
        private final double speed;

        private final int speedPow = 3;

        private final SlewRateLimiter xLimiter;
        private final SlewRateLimiter yLimiter;
        private final SlewRateLimiter rotLimiter;


        public DriveContinuous (DrivetrainSubsystem drivetrain, GamepadEx driver, MultipleTelemetry multipleTelemetry, double speed) {
            this.drivetrain = drivetrain;
            this.driver = driver;
            this.multipleTelemetry = multipleTelemetry;
            this.speed = speed;

            xLimiter = new SlewRateLimiter(10, TimeTracker.getTime());
            yLimiter = new SlewRateLimiter(10, TimeTracker.getTime());
            rotLimiter = new SlewRateLimiter(10, TimeTracker.getTime());

            if (RobotState.getInstance().isBlueTeam() != null && RobotState.getInstance().isBlueTeam()) {
                // Blue team: field forward at -90 degrees
                RobotState.getInstance().setHeadingOffset(Rotation2d.fromDegrees(-90));
            } else if (RobotState.getInstance().isBlueTeam() != null && !RobotState.getInstance().isBlueTeam()){
                // Red team: field forward at +90 degrees
                RobotState.getInstance().setHeadingOffset(Rotation2d.fromDegrees(90));
            } else {
                RobotState.getInstance().setHeadingOffset(Rotation2d.fromDegrees(0));
            }

            addRequirements(drivetrain);
        }

        /**
         * Reads joystick input every cycle and applies it as a drive power.
         */
        @Override
        public void execute() {
            double time = TimeTracker.getTime();

            // Apply cubic control before rate limiting
            double xInput = Math.pow(driver.getLeftY(), speedPow);
            double yInput = Math.pow(-driver.getLeftX(), speedPow);
            double rotInput = Math.pow(-driver.getRightX(), speedPow);

            // Apply slew rate limiting and speed scaling
            double xSpeed = xLimiter.calculate(xInput, time) * speed;
            double ySpeed = yLimiter.calculate(yInput, time) * speed;
            double rotSpeed = rotLimiter.calculate(rotInput, time) * speed;

            drivetrain.driveFieldCentric(
                    xSpeed,
                    ySpeed,
                    rotSpeed,
                    RobotState.getInstance().getHeadingOffset().getRadians()
            );

            multipleTelemetry.addData("xSpeed", xSpeed);
            multipleTelemetry.addData("ySpeed", ySpeed);
            multipleTelemetry.addData("rotSpeed", rotSpeed);
        }

        /**
         * Stops the drivetrain when the command ends, whether completed or interrupted.
         *
         * @param isInterrupted true if the command was canceled or OpMode ended
         */
        @Override
        public void end(boolean isInterrupted) {
            // Sets the drive powers to 0
            drivetrain.resumeManualDrive();
        }

}
