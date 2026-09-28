package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class FlywheelConfig {
    public static final double MOTOR_TPR = 0;//todo

    public static final double GEAR_RATIO = 1/3;

    public static final double TICKS_PER_REV = MOTOR_TPR / GEAR_RATIO;
}
