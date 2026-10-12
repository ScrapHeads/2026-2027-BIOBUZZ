package org.firstinspires.ftc.teamcode.config.pedro;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.util.PIDFValues;

@Config
public class TurretPosConfig {
    public static double targetPos = 0;
    public static double targetPower = 0;

    public static boolean enablePID = false;

    public static PIDFValues pidfValues = new PIDFValues(
            0.0, 0.0, 0.0, 0.0
    );
}
