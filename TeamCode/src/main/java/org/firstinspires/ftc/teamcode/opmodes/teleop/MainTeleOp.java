package org.firstinspires.ftc.teamcode.opmodes.teleop;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import static com.seattlesolvers.solverslib.gamepad.GamepadKeys.Button.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.drive.DriveContinuous;
import org.firstinspires.ftc.teamcode.commands.intake.SetPowerIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.turretstopper.SetPosTurretStopperCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commands.transfer.SetPowerTransferCommand;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretStopperSubsystem;


@TeleOp(name = "MainTeleOp", group = "ScrapHeads")
public class MainTeleOp extends CommandOpMode {

    public Robot robot;

    public HardwareMap hm;
    public Telemetry tele;

    public GamepadEx driver1;
    public GamepadEx driver2;

    @Override
    public void initialize() {
        this.tele = telemetry;
        this.hm = hardwareMap;

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        robot = new Robot(hardwareMap, telemetry);

        assignControls();
    }

    @Override
    public void run() {
        super.run(); // VERY IMPORTANT: runs the CommandScheduler on every loop cycle

        // Flushes telemetry updates to BOTH Driver Hub and FTC Dashboard
        robot.getTelemetry().update();
    }

    public void  assignControls() {
        robot.drivetrain.setDefaultCommand(new DriveContinuous(
                robot.drivetrain, driver1, robot.multipleTelemetry, 1
        ));

        new Trigger(() -> driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > .1)
                .whenActive(new ParallelCommandGroup(
                        new SetPowerIntakeCommand(robot.intake, IntakeSubsystem.INTAKE_POWER),
                        new SetPowerTransferCommand(robot.transfer, TransferSubsystem.TRANSFER_IN_POWER)))
                .whenInactive(new ParallelCommandGroup(
                        new SetPowerIntakeCommand(robot.intake, 0),
                        new SetPowerTransferCommand(robot.transfer, 0)));

        driver1.getGamepadButton(DPAD_UP)
                .whenPressed(new SetPowerTransferCommand(robot.transfer, TransferSubsystem.TRANSFER_IN_POWER));
        driver1.getGamepadButton(DPAD_DOWN)
                .whenPressed(new SetPowerTransferCommand(robot.transfer, TransferSubsystem.TRANSFER_OUT_POWER));

        new Trigger(() -> driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > .1)
                .whenActive(new ParallelCommandGroup(
                        new SetPowerIntakeCommand(robot.intake, IntakeSubsystem.OUTTAKE_POWER),
                        new SetPowerTransferCommand(robot.transfer, TransferSubsystem.TRANSFER_OUT_POWER)
                ))
                .whenInactive(new ParallelCommandGroup(
                        new SetPowerIntakeCommand(robot.intake, 0),
                        new SetPowerTransferCommand(robot.transfer, 0)));

        driver1.getGamepadButton(Y)
                .whenPressed(new ParallelCommandGroup(
                        new SetPowerTransferCommand(robot.transfer, 0),
                        new SetPowerIntakeCommand(robot.intake, 0)
                ));

        driver1.getGamepadButton(A)
                        .whenPressed(new ParallelCommandGroup(
                                new SetPowerIntakeCommand(robot.intake, IntakeSubsystem.INTAKE_POWER),
                                new SetPowerTransferCommand(robot.transfer, TransferSubsystem.TRANSFER_IN_POWER),
                                new SetPosTurretStopperCommand(robot.turretStopper, TurretStopperSubsystem.OPEN_POS)
                        ))
                        .whenReleased(new SetPosTurretStopperCommand(robot.turretStopper, TurretStopperSubsystem.CLOSED_POS));

//        driver1.getGamepadButton(X)
//                .whenPressed(new InstantCommand(() -> robot.vision.setPipeline(VisionConstants.GAME_ELEMENT_TRACKER_PIPELINE)));
//        driver1.getGamepadButton(B)
//                .whenPressed(new InstantCommand(() -> robot.vision.setPipeline(VisionConstants.APRIL_TAG_PIPELINE)));

        driver1.getGamepadButton(X)
                .whenPressed(new SetPosTurretStopperCommand(robot.turretStopper, TurretStopperSubsystem.OPEN_POS));
        driver1.getGamepadButton(B)
                .whenPressed(new SetPosTurretStopperCommand(robot.turretStopper, TurretStopperSubsystem.CLOSED_POS));
    }
}
