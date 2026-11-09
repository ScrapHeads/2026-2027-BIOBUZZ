package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static com.seattlesolvers.solverslib.gamepad.GamepadKeys.Button.*;

import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.drive.DriveContinuous;
import org.firstinspires.ftc.teamcode.commands.flowerRamp.ToggleFlowerRampCommand;
import org.firstinspires.ftc.teamcode.commands.groups.IntakeTransferSetPower;
import org.firstinspires.ftc.teamcode.commands.intake.SetPowerIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.intakeBlocker.ToggelBlockerCommand;
import org.firstinspires.ftc.teamcode.commands.transfer.SetPowerTransferCommand;
import org.firstinspires.ftc.teamcode.commands.turretstopper.SetPosTurretStopperCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretStopperSubsystem;

public class TeleOpControls {
    public static void assignControls(Robot robot, GamepadEx driver1, GamepadEx driver2) {
        robot.drivetrain.setDefaultCommand(new DriveContinuous(
                robot.drivetrain, driver1, robot.multipleTelemetry, 1
        ));

        new Trigger(() -> driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > .1)
                .whenActive(new ParallelCommandGroup(
                        new SetPowerIntakeCommand(robot.intake, IntakeSubsystem.INTAKE_POWER),
                        new SetPowerTransferCommand(robot.transfer, TransferSubsystem.TRANSFER_IN_POWER)))
                .whenInactive(new IntakeTransferSetPower(robot.intake, robot.transfer, IntakeSubsystem.INTAKE_POWER, TransferSubsystem.TRANSFER_IN_POWER));

        driver1.getGamepadButton(RIGHT_BUMPER)
                .whenPressed(new SetPowerIntakeCommand(robot.intake, -.5))
                .whenReleased(new SetPowerIntakeCommand(robot.intake, 0));

        new Trigger(() -> driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > .1)
                .whenActive(new ParallelCommandGroup(
                        new SetPowerIntakeCommand(robot.intake, IntakeSubsystem.OUTTAKE_POWER),
                        new SetPowerTransferCommand(robot.transfer, TransferSubsystem.TRANSFER_OUT_POWER)
                ))
                .whenInactive(new IntakeTransferSetPower(robot.intake,robot.transfer,IntakeSubsystem.OUTTAKE_POWER, TransferSubsystem.TRANSFER_OUT_POWER));

        driver1.getGamepadButton(LEFT_BUMPER)
                .whenPressed(new ParallelCommandGroup(
                        new SetPowerIntakeCommand(robot.intake, IntakeSubsystem.INTAKE_POWER),
                        new SetPowerTransferCommand(robot.transfer, TransferSubsystem.TRANSFER_OUT_POWER_SLOW)
                ))
                .whenReleased(new IntakeTransferSetPower(robot.intake,robot.transfer,IntakeSubsystem.INTAKE_POWER,TransferSubsystem.TRANSFER_OUT_POWER_SLOW));

        driver1.getGamepadButton(DPAD_UP)
                .whenPressed(new SetPowerTransferCommand(robot.transfer, TransferSubsystem.TRANSFER_IN_POWER));
        driver1.getGamepadButton(DPAD_DOWN)
                .whenPressed(new SetPowerTransferCommand(robot.transfer, TransferSubsystem.TRANSFER_OUT_POWER));

        driver1.getGamepadButton(A)
                .whenPressed(new ParallelCommandGroup(
                        new SetPowerIntakeCommand(robot.intake, IntakeSubsystem.INTAKE_LAUNCH_POWER),
                        new SetPowerTransferCommand(robot.transfer, TransferSubsystem.TRANSFER_LAUNCH_POWER),
                        new SetPosTurretStopperCommand(robot.turretStopper, TurretStopperSubsystem.OPEN_POS)
                ))
                .whenReleased(new ParallelCommandGroup(
                        new SetPosTurretStopperCommand(robot.turretStopper, TurretStopperSubsystem.CLOSED_POS),
                        new SetPowerIntakeCommand(robot.intake, 0),
                        new SetPowerTransferCommand(robot.transfer, 0))
                );

        driver1.getGamepadButton(Y)
                .whenPressed(new ToggleFlowerRampCommand(robot.ramp));
        driver1.getGamepadButton(B)
                .whenPressed(new ToggelBlockerCommand(robot.intakeBlocker));

//        driver1.getGamepadButton(X)
//                .whenPressed(new InstantCommand(() -> robot.vision.setPipeline(VisionConstants.GAME_ELEMENT_TRACKER_PIPELINE)));
//        driver1.getGamepadButton(B)
//                .whenPressed(new InstantCommand(() -> robot.vision.setPipeline(VisionConstants.APRIL_TAG_PIPELINE)));
//
//        driver1.getGamepadButton(X)
//                .whenPressed(new SetPosTurretStopperCommand(robot.turretStopper, TurretStopperSubsystem.OPEN_POS));
//        driver1.getGamepadButton(B)
//                .whenPressed(new SetPosTurretStopperCommand(robot.turretStopper, TurretStopperSubsystem.CLOSED_POS));
    }
}
