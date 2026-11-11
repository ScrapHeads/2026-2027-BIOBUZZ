package org.firstinspires.ftc.teamcode.robotTests.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.firstinspires.ftc.teamcode.config.FlywheelConfig;
import org.firstinspires.ftc.teamcode.util.FlywheelInterpolation;
import org.junit.jupiter.api.Test;

public class FlywheelSubsystemTest {

    @Test
    void distanceTableWorksBeforeManualRefresh() {
        double[][] table = FlywheelConfig.FLYWHEEL_TABLE;
        FlywheelInterpolation interpolation = new FlywheelInterpolation();
        assertEquals(table[0][1], interpolation.getRPM(table[0][0]), 1e-8);
        double midpoint = (table[0][0] + table[1][0]) / 2.0;
        assertEquals((table[0][1] + table[1][1]) / 2.0,
                interpolation.getRPM(midpoint), 1e-8);
    }
}
