package org.firstinspires.ftc.teamcode.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

public class TimeTrackerTest {

    @Test
    @DisplayName("Test setOffset resets relative time near zero")
    public void testSetOffsetAndGetTime() {
        TimeTracker.setOffset();
        double initialTime = TimeTracker.getTime();

        // Immediately after setOffset, time should be non-negative and very close to 0 seconds
        assertTrue(initialTime >= 0.0, "Time should be >= 0");
        assertTrue(initialTime < 0.1, "Time immediately after offset should be close to 0");
    }

    @Test
    @DisplayName("Test time increases over duration")
    public void testTimeElapses() throws InterruptedException {
        TimeTracker.setOffset();
        double t1 = TimeTracker.getTime();

        Thread.sleep(50); // sleep for 50 milliseconds

        double t2 = TimeTracker.getTime();
        double dt = t2 - t1;

        // Expect dt to be around 0.05 seconds (50ms)
        assertTrue(dt >= 0.04, "Elapsed time should reflect duration slept (>= 40ms)");
        assertTrue(dt <= 0.20, "Elapsed time should not exceed reasonable sleep duration");
    }

    @Test
    @DisplayName("Test convertTime applies offset correctly")
    public void testConvertTime() {
        TimeTracker.setOffset();
        double nowSeconds = System.currentTimeMillis() / 1000.0;
        double converted = TimeTracker.convertTime(nowSeconds);

        // Convert time should equal current getTime()
        double current = TimeTracker.getTime();
        assertEquals(current, converted, 0.05);
    }
}
