package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.util.PIDFValues;

@Config
public class TurretPosConfig {
    // Target turret angle in degrees, using the subsystem's startup reference.
    public static double targetAngleDegrees = 0;
    public static double targetPower = 0;

    public static boolean enablePID = true;
    public static boolean solveAngle = true;

    // TODO: Tune turret PIDF gains with the position error measured in degrees.
    public static PIDFValues pidfValues = new PIDFValues(
            0.0, 0.0, 0.0, 0.0
    );
}
