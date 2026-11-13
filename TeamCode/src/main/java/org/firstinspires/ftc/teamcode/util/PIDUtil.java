package org.firstinspires.ftc.teamcode.util;

import com.seattlesolvers.solverslib.controller.PIDFController;

public class PIDUtil {
    public static void updatePIDController (PIDFController controller, PIDFValues values) {
        if (values.hasChanged()) {
            controller.setPIDF(
                    values.kP,
                    values.kI,
                    values.kD,
                    values.kF
            );
            // Clear accumulated history once when gains change, not on every loop.
            controller.reset();
            values.setLasts();
        }
    }
}
