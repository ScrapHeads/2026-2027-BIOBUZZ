package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.util.PIDFValues;

@Config
public class TurretPosConfig {
    // Target turret angle in degrees, using the subsystem's startup reference.
    public static double targetAngleDegrees = 0;
    public static double targetPower = 0;

    public static boolean enablePID = false;
    public static boolean solveAngle = false;
    // Show hive outlines and the solved shot on the FTC Dashboard field overlay.
    public static boolean showAimDashboard = true;

    // TODO: Tune turret PIDF gains with the position error measured in degrees.
    public static PIDFValues pidfValues = new PIDFValues(
            0.031, 0.0, 0.001, 0.0
    );
}
