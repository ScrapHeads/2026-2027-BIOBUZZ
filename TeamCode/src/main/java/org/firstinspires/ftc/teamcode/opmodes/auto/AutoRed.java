package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.flywheel.SetRPMFlywheelCommand;
import org.firstinspires.ftc.teamcode.commands.intake.SetPowerIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.transfer.SetPowerTransferCommand;
import org.firstinspires.ftc.teamcode.commands.turretstopper.SetPosTurretStopperCommand;
import org.firstinspires.ftc.teamcode.pathing.paths.RedPath;
import org.firstinspires.ftc.teamcode.pathing.paths.RedTestPath;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretStopperSubsystem;

@Autonomous
public class AutoRed extends CommandOpMode {
    public Robot robot;

    public HardwareMap hm;
    public Telemetry tele;

    private RedPath pathSet;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        this.tele = telemetry;
        this.hm = hardwareMap;

        pathSet = new RedPath();

        robot = new Robot(hardwareMap, telemetry);
        robot.drivetrain.follower.setPose(pathSet.moveToShootPreLoadedPoint1);

    }

    public SequentialCommandGroup launchSequence () {
        return new SequentialCommandGroup (
                new ParallelCommandGroup(
                        new SetPowerIntakeCommand(robot.intake, IntakeSubsystem.INTAKE_POWER),
                        new SetPowerTransferCommand(robot.transfer, TransferSubsystem.TRANSFER_IN_POWER),
                        new SetPosTurretStopperCommand(robot.turretStopper, TurretStopperSubsystem.OPEN_POS)
                ),
                new WaitCommand(1500),
                new ParallelCommandGroup(
                    new SetPowerIntakeCommand(robot.intake, 0),
                    new SetPowerTransferCommand(robot.transfer, 0),
                    new SetPosTurretStopperCommand(robot.turretStopper, TurretStopperSubsystem.CLOSED_POS))
        );
    }

    @Override
    public void preRun() {
        Follower follower = robot.drivetrain.follower;
        new SequentialCommandGroup(
                new SetRPMFlywheelCommand(robot.flyWheel, 600),
                new FollowPathCommand(follower, pathSet.moveToShootPreLoaded(), .5),

                new WaitCommand(3000),

                launchSequence(),

                new ParallelCommandGroup(
                        new FollowPathCommand(follower, pathSet.pickupGarden(), .6),
                        new SetPowerIntakeCommand(robot.intake, IntakeSubsystem.INTAKE_POWER),
                        new SetPowerTransferCommand(robot.transfer, TransferSubsystem.TRANSFER_IN_POWER)
                ),

                new WaitCommand(500),

                new ParallelCommandGroup(
                        new FollowPathCommand(follower, pathSet.launchGarden()),
                        new SetPowerIntakeCommand(robot.intake, 0),
                        new SetPowerTransferCommand(robot.transfer, 0)
                ),

                launchSequence(),

                new ParallelCommandGroup(
                        new FollowPathCommand(follower, pathSet.pickUpLeftFlower(), 1),
                        new SetPowerIntakeCommand(robot.intake, IntakeSubsystem.INTAKE_POWER),
                        new SetPowerTransferCommand(robot.transfer, TransferSubsystem.TRANSFER_IN_POWER)
                ),

                new WaitCommand(1000),

                new ParallelCommandGroup(
                        new FollowPathCommand(follower, pathSet.launchLeftFlower()),
                        new SetPowerIntakeCommand(robot.intake, 0),
                        new SetPowerTransferCommand(robot.transfer, 0)
                ),

                launchSequence(),

                new ParallelCommandGroup(
                        new FollowPathCommand(follower, pathSet.pickUpBackFlower(), 1),
                        new SetPowerIntakeCommand(robot.intake, IntakeSubsystem.INTAKE_POWER),
                        new SetPowerTransferCommand(robot.transfer, TransferSubsystem.TRANSFER_IN_POWER)
                ),

                new WaitCommand(1000),

                new ParallelCommandGroup(
                        new FollowPathCommand(follower, pathSet.launchBackFlower()),
                        new SetPowerIntakeCommand(robot.intake, 0),
                        new SetPowerTransferCommand(robot.transfer, 0)
                ),

                launchSequence(),

                new FollowPathCommand(follower, pathSet.park())

        ).schedule();
    }

    @Override
    public void run() {
        super.run();

        robot.getTelemetry().addLine("Running");
        robot.getTelemetry().update();
    }
}
