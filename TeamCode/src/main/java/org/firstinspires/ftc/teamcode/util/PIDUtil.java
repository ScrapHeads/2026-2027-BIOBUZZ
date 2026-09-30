package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.config.FlywheelConfig.pidfValues;

import com.seattlesolvers.solverslib.controller.PIDFController;

public class PIDUtil {
    public static void updatePIDController (PIDFController controller, PIDFValues values) {
        if (values.hasChanged()) {
            controller.setPIDF(
                    pidfValues.kP,
                    pidfValues.kI,
                    pidfValues.kD,
                    pidfValues.kF
            );
            values.setLasts();
        }
    }
}
