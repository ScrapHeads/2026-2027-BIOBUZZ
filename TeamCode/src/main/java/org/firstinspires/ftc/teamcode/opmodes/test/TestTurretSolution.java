package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RilLib.Math.Units;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.config.FlywheelConfig;
import org.firstinspires.ftc.teamcode.config.TurretPosConfig;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlyWheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretRotSubsystem;

/** Drivetrain, turret aiming, and dashboard-controlled flywheel test. */
@Config
@TeleOp(name = "TestTurretSolution", group = "Test")
public class TestTurretSolution extends LinearOpMode {
    public static boolean BLUE = false;
    public static RobotState.Goal GOAL = RobotState.Goal.SOUTH;
    public static double DRIVE_SPEED = 0.35;

    @Override
    public void runOpMode() throws InterruptedException {
        MultipleTelemetry output = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry());
        RobotState state = RobotState.getInstance();
        DrivetrainSubsystem drivetrain = new DrivetrainSubsystem(hardwareMap, output);
        TurretRotSubsystem turret = new TurretRotSubsystem(hardwareMap, output);
        FlyWheelSubsystem flywheel = new FlyWheelSubsystem(hardwareMap, output);
        // Override the flywheel constructor's presets for manual dashboard tuning.
        flywheel.setTargetRPM(0.0);
        flywheel.setEnablePID(false);
        FlywheelConfig.targetPower = 0.0;
        FlywheelConfig.setByDistance = false;
        boolean previousShowAimDashboard = TurretPosConfig.showAimDashboard;

        try {
            drivetrain.follower.setPose(new Pose(8.5, 9.0, Units.degreesToRadians(90.0)));
            state.setTeam(BLUE);
            state.setCurrentGoal(GOAL);
            TurretPosConfig.showAimDashboard = true;

            output.addLine("Start pose: (8.5 in, 9 in, 90 deg)");
            output.addLine("Left stick: drive; right stick X: turn; A: toggle north/south.");
            output.addLine("Set alliance and goal in TestTurretSolution dashboard config.");
            output.addLine("Turret PID and automatic aiming use the TurretPosConfig settings.");
            output.addLine("FlywheelConfig: set targetRPM, pidfValues, and enablePID in dashboard.");
            output.addLine("Flywheel starts at 0 RPM with PID and distance mode disabled.");
            output.update();

            waitForStart();
            boolean previousA = gamepad1.a;
            while (opModeIsActive()) {
                state.setTeam(BLUE);
                if (gamepad1.a && !previousA) {
                    GOAL = GOAL == RobotState.Goal.NORTH
                            ? RobotState.Goal.SOUTH : RobotState.Goal.NORTH;
                }
                previousA = gamepad1.a;
                state.setCurrentGoal(GOAL);

                double speed = Math.max(0.0, Math.min(1.0, DRIVE_SPEED));
                drivetrain.drive(-gamepad1.left_stick_y * speed,
                        -gamepad1.left_stick_x * speed,
                        -gamepad1.right_stick_x * speed);
                // Update localization before the turret reads the shared aiming snapshot.
                drivetrain.periodic();
                turret.periodic();
                flywheel.periodic();

                output.addData("Flywheel target RPM", flywheel.getTargetRPM());
                output.addData("Flywheel PID enabled", flywheel.getEnablePID());
                output.addData("Alliance", BLUE ? "Blue" : "Red");
                output.addData("Goal", state.getCurrentGoal());
                output.addData("Target cell", state.getTargetCell());
                output.addData("Aim solution available", state.getAimSolution().isPresent());
                output.addLine("A: toggle north/south");
                output.update();
                sleep(20);
            }
        } finally {
            flywheel.setPower(0.0);
            turret.setPower(0.0);
            drivetrain.stop();
            TurretPosConfig.showAimDashboard = previousShowAimDashboard;
        }
    }
}
