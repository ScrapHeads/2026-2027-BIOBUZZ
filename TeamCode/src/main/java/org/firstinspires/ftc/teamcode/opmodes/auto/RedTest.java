package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.flywheel.SetRPMFlywheelCommand;
import org.firstinspires.ftc.teamcode.pathing.paths.RedTestPath;

import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

@Autonomous
public class RedTest extends CommandOpMode {

    public Robot robot;

    public HardwareMap hm;
    public Telemetry tele;

    private RedTestPath pathSet;

    @Override
    public void initialize() {
        this.tele = telemetry;
        this.hm = hardwareMap;

        pathSet = new RedTestPath();

        robot = new Robot(hardwareMap, telemetry);
        robot.drivetrain.follower.setPose(pathSet.startPoint1);
    }

    @Override
    public void preRun() {
        Follower follower = robot.drivetrain.follower;
        new SequentialCommandGroup(
                new SetRPMFlywheelCommand(robot.flyWheel, 600),
                new WaitCommand(1000),
                new FollowPathCommand(follower, pathSet.start())
        ).schedule();
    }

    @Override
    public void run() {
        robot.drivetrain.follower.update();
        super.run();

        robot.getTelemetry().addLine("Running");
        robot.getTelemetry().update();
    }
}
