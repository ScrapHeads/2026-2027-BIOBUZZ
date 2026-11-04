package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RilLib.Math.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.commands.intake.SetPowerIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.transfer.SetPowerTransferCommand;
import org.firstinspires.ftc.teamcode.commands.turretstopper.SetPosTurretStopperCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretStopperSubsystem;

public abstract class BaseAuto extends CommandOpMode {

    public Robot robot;
    public HardwareMap hm;
    public Telemetry tele;

    protected abstract boolean isBlueTeam();

    protected abstract Pose getStartingPose();

    protected abstract void initPaths();

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        this.tele = telemetry;
        this.hm = hardwareMap;

        robot = new Robot(hardwareMap, telemetry);

        initPaths();

        Pose startPose = getStartingPose();
        if (startPose != null) {
            robot.drivetrain.follower.setPose(startPose);
            RobotState.getInstance().setTeam(isBlueTeam());
            RobotState.getInstance().setAll(
                    new Pose2d(startPose.x(), startPose.y(), new Rotation2d(startPose.heading())),
                    isBlueTeam(),
                    new ChassisSpeeds()
            );
            robot.state.save();
        }
    }

    public SequentialCommandGroup launchSequence() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new SetPowerIntakeCommand(robot.intake, IntakeSubsystem.INTAKE_MID_POWER),
                        new SetPowerTransferCommand(robot.transfer, TransferSubsystem.TRANSFER_IN_MID),
                        new SetPosTurretStopperCommand(robot.turretStopper, TurretStopperSubsystem.OPEN_POS)
                ),
                new WaitCommand(1500),
                new ParallelCommandGroup(
                        new SetPowerIntakeCommand(robot.intake, 0),
                        new SetPowerTransferCommand(robot.transfer, 0),
                        new SetPosTurretStopperCommand(robot.turretStopper, TurretStopperSubsystem.CLOSED_POS)
                )
        );
    }

    @Override
    public void run() {
        super.run();

        Pose currentPose = robot.drivetrain.getPose();
        if (currentPose != null) {
            RobotState.getInstance().setAll(
                    new Pose2d(currentPose.x(), currentPose.y(), new Rotation2d(currentPose.heading())),
                    isBlueTeam(),
                    new ChassisSpeeds()
            );
            robot.state.save();
        }

        robot.getTelemetry().addLine("Running");
        robot.getTelemetry().update();
    }
}
