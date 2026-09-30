package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.util.PIDFValues;

@Config
public class FlywheelConfig {
    public static double targetRPM = 0;
    public static double targetPower = 0;
    public static boolean enablePID = false;

    public static PIDFValues pidfValues = new PIDFValues(
            0.0, 0.0, 0.0, 0.0
    );
}
