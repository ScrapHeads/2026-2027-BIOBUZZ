package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;

@Config
public class RobotConstants {
    public static final double CENTER_TO_FRONT_OFFSET_CM = 0.0; //TODO Set value

    public static boolean isFrontGoal = false;

    public static final Pose2d BLUE_FRONT_GOAL = new Pose2d();
    public static final Pose2d RED_FRONT_GOAL = new Pose2d();
    public static final Pose2d BLUE_BACK_GOAL = new Pose2d();
    public static final Pose2d RED_BACK_GOAL = new Pose2d();
}
