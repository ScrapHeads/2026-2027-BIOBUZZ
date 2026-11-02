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
import org.firstinspires.ftc.teamcode.RilLib.Math.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.commands.flowerRamp.setFlowerRampPosCommand;
import org.firstinspires.ftc.teamcode.commands.flywheel.SetRPMFlywheelCommand;
import org.firstinspires.ftc.teamcode.commands.intake.SetPowerIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.intakeBlocker.setIntakeBlockerPosBothCommand;
import org.firstinspires.ftc.teamcode.commands.transfer.SetPowerTransferCommand;
import org.firstinspires.ftc.teamcode.commands.turretstopper.SetPosTurretStopperCommand;
import org.firstinspires.ftc.teamcode.pathing.paths.RedPath;
import org.firstinspires.ftc.teamcode.subsystems.FlowerRampSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeBlockerSubsystem;
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

        RobotState.getInstance().setTeam(false);
        com.pedropathing.math.Pose startPose = pathSet.moveToShootPreLoadedPoint1;
        RobotState.getInstance().setAll(
                new Pose2d(startPose.x(), startPose.y(), new Rotation2d(startPose.heading())),
                false,
                new ChassisSpeeds()
        );
        robot.state.save();
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
                new ParallelCommandGroup(
                        new SetRPMFlywheelCommand(robot.flyWheel, 590),
                        new setIntakeBlockerPosBothCommand(robot.intakeBlocker,
                                IntakeBlockerSubsystem.LEFT_OPEN, IntakeBlockerSubsystem.RIGHT_OPEN)
                ),

                new FollowPathCommand(follower, pathSet.moveToShootPreLoaded(), .3),

                new setIntakeBlockerPosBothCommand(robot.intakeBlocker,
                        IntakeBlockerSubsystem.LEFT_LOCK, IntakeBlockerSubsystem.RIGHT_LOCK),

                new WaitCommand(1500),

                launchSequence(),

                new ParallelCommandGroup(
                        new FollowPathCommand(follower, pathSet.prePickupGarden()),
                        new SetPowerIntakeCommand(robot.intake, IntakeSubsystem.OUTTAKE_POWER),
                        new SetPowerTransferCommand(robot.transfer, TransferSubsystem.TRANSFER_OUT_POWER)
                ),

                new ParallelCommandGroup(
                        new FollowPathCommand(follower, pathSet.pickUpGarden(),.4).withTimeout(1500),
                        new SetPowerIntakeCommand(robot.intake, IntakeSubsystem.INTAKE_POWER),
                        new SetPowerTransferCommand(robot.transfer, TransferSubsystem.TRANSFER_IN_POWER)
                ),
//
                new WaitCommand(500),

                new ParallelCommandGroup(
                        new FollowPathCommand(follower, pathSet.launchGarden(), .9),
                        new SetPowerIntakeCommand(robot.intake, 0),
                        new SetPowerTransferCommand(robot.transfer, 0)
                ),

                launchSequence(),
//
                new FollowPathCommand(follower, pathSet.prePickUpLeftFlower()),
//
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, pathSet.pickUpLeftFlower(), .4).withTimeout(1500),
                        new setFlowerRampPosCommand(robot.ramp, FlowerRampSubsystem.DOWN_POS),
                        new SetPowerIntakeCommand(robot.intake, IntakeSubsystem.INTAKE_POWER),
                        new SetPowerTransferCommand(robot.transfer, TransferSubsystem.TRANSFER_IN_POWER)
                ),

                new WaitCommand(1150),

                new ParallelCommandGroup(
                        new FollowPathCommand(follower, pathSet.launchLeftFlower(), .7),
                        new WaitCommand(500).andThen(
                            new setFlowerRampPosCommand(robot.ramp, FlowerRampSubsystem.UP_POS),
                            new SetPowerIntakeCommand(robot.intake, 0),
                            new SetPowerTransferCommand(robot.transfer, 0)
                        )
                ),

                launchSequence(),

                new ParallelCommandGroup(
                        new FollowPathCommand(follower, pathSet.prePickUpBackFlower(), .4),
                        new SetPowerIntakeCommand(robot.intake, IntakeSubsystem.OUTTAKE_POWER),
                        new SetPowerTransferCommand(robot.transfer, TransferSubsystem.TRANSFER_OUT_POWER)
                ),

                new ParallelCommandGroup(
                        new FollowPathCommand(follower, pathSet.pickUpBackFlower(), .3).withTimeout(750),
                        new setFlowerRampPosCommand(robot.ramp, FlowerRampSubsystem.DOWN_POS),
                        new SetPowerIntakeCommand(robot.intake, IntakeSubsystem.INTAKE_POWER),
                        new SetPowerTransferCommand(robot.transfer, TransferSubsystem.TRANSFER_IN_POWER)
                ),
//
                new WaitCommand(1150),

                new ParallelCommandGroup(
                        new FollowPathCommand(follower, pathSet.launchBackFlower(), .35),
                        new WaitCommand(500).andThen(
                                new setFlowerRampPosCommand(robot.ramp, FlowerRampSubsystem.UP_POS),
                                new SetPowerIntakeCommand(robot.intake, 0),
                                new SetPowerTransferCommand(robot.transfer, 0)
                        )
                ),
//
                launchSequence(),
//
                new FollowPathCommand(follower, pathSet.park())

        ).schedule();
    }

    @Override
    public void run() {
        super.run();

        com.pedropathing.math.Pose currentPose = robot.drivetrain.getPose();
        RobotState.getInstance().setAll(
                new Pose2d(currentPose.x(), currentPose.y(), new Rotation2d(currentPose.heading())),
                false,
                new ChassisSpeeds()
        );
        robot.state.save();

        robot.getTelemetry().addLine("Running");
        robot.getTelemetry().update();
    }
}
