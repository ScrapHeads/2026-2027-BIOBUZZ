package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.flywheel.SetRPMFlywheelCommand;
import org.firstinspires.ftc.teamcode.pathing.paths.RedTestPath;

import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

@Autonomous
public class RedTest extends OpMode{

    public Robot robot;

    public HardwareMap hm;
    public Telemetry tele;

    public RedTestPath path = new RedTestPath();

    @Override
    public void init() {
        this.tele = telemetry;
        this.hm = hardwareMap;

        robot = new Robot(hardwareMap, telemetry);
    }

    @Override
    public void start() {
        Follower follower = robot.drivetrain.follower;
        new SequentialCommandGroup(
                new SetRPMFlywheelCommand(robot.flyWheel, 600),
                new FollowPathCommand(follower, path.start()),

        ).schedule();

    }
    @Override
    public void loop() {

    }
}
