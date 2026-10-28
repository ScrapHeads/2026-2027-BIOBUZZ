package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.RilLib.Math.Interpolation.InterpolatingTreeMap;
import org.firstinspires.ftc.teamcode.util.PIDFValues;

@Config
public class FlywheelConfig {
    public static double targetRPM = 0.0;
    public static double targetPower = 0.0;
    public static boolean enablePID = false;
    public static boolean setByDistance = false;

    public static PIDFValues pidfValues = new PIDFValues(
            0.023, 0.0, 0.0, .00035
    );

    public static boolean REFRESH_TABLE = false;

    public static double[][] FLYWHEEL_TABLE = {
            {0, 2000},
            {25, 2300},
            {50, 2600},
            {75, 2900},
            {100, 3200},
            {125, 3500},
            {150, 3800},
            {175, 4100},
            {200, 4400}
    };

}
