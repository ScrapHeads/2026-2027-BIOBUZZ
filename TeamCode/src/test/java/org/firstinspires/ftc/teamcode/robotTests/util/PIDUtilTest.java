package org.firstinspires.ftc.teamcode.robotTests.util;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.util.PIDFValues;
import org.firstinspires.ftc.teamcode.util.PIDUtil;
import org.junit.jupiter.api.Test;

class PIDUtilTest {
    private static class TrackingController extends PIDFController {
        int resets;

        TrackingController() {
            super(0, 0, 0, 0);
        }

        void seedIntegral(double integral) {
            totalError = integral;
        }

        double integral() {
            return totalError;
        }

        @Override
        public void reset() {
            super.reset();
            resets++;
        }
    }

    @Test
    void appliesSuppliedGainsAndClearsHistoryOnce() {
        TrackingController controller = new TrackingController();
        PIDFValues values = new PIDFValues(0, 0, 0, 0);
        controller.seedIntegral(0.5);
        values.kP = 0.1;
        values.kI = 0.2;
        values.kD = 0.3;
        values.kF = 0.4;

        PIDUtil.updatePIDController(controller, values);

        assertArrayEquals(new double[]{0.1, 0.2, 0.3, 0.4}, controller.getCoefficients(), 1e-12);
        assertEquals(0.0, controller.integral(), 1e-12);
        assertEquals(1, controller.resets);
        assertFalse(values.hasChanged());

        controller.seedIntegral(0.25);
        PIDUtil.updatePIDController(controller, values);
        assertEquals(1, controller.resets);
        assertEquals(0.25, controller.integral(), 1e-12);
    }

    @Test
    void zeroingGainsProducesZeroOutput() {
        TrackingController controller = new TrackingController();
        PIDFValues values = new PIDFValues(0, 0, 0, 0);
        values.kP = 0.1;
        PIDUtil.updatePIDController(controller, values);
        assertEquals(1.0, controller.calculate(0, 10), 1e-12);

        values.kP = 0.0;
        PIDUtil.updatePIDController(controller, values);

        assertArrayEquals(new double[]{0, 0, 0, 0}, controller.getCoefficients(), 1e-12);
        assertEquals(0.0, controller.calculate(0, 10), 1e-12);
        assertEquals(2, controller.resets);
    }
}
