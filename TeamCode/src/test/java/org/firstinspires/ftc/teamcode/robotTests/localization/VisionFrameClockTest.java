package org.firstinspires.ftc.teamcode.robotTests.localization;

import org.firstinspires.ftc.teamcode.vision.localization.VisionFrameClock;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;
import static org.firstinspires.ftc.teamcode.vision.localization.VisionFrameClock.Status.*;

class VisionFrameClockTest {
    @Test void subtractsLatencyAndConsumesEachPhysicalFrameOnlyOnce() {
        VisionFrameClock clock = new VisionFrameClock();
        VisionFrameClock.Reading first = clock.read(100, 10, 10000, 9990, 10, 20, 0.02, 0.35);
        assertEquals(NEW, first.status);
        assertEquals(9.94, first.captureTime, 1e-9);
        assertEquals(DUPLICATE, clock.read(100, 10.1, 10100, 10100, 10, 20, 0.02, 0.35).status);
    }
    @Test void rejectsStaleFramesAndCameraRestartThenRecovers() {
        VisionFrameClock clock = new VisionFrameClock();
        assertEquals(STALE, clock.read(100, 10, 10000, 9000, 10, 20, 0.02, 0.35).status);
        assertEquals(RESTART, clock.read(1, 10.1, 10100, 10100, 10, 20, 0.02, 0.35).status);
        assertEquals(NEW, clock.read(2, 10.2, 10200, 10200, 10, 20, 0.02, 0.35).status);
    }
    @Test void detectsWallClockJumpAndMissingLatency() {
        VisionFrameClock clock = new VisionFrameClock();
        clock.read(1, 10, 10000, 10000, 10, 20, 0.02, 0.35);
        assertEquals(RESTART, clock.read(2, 10.1, 11100, 11100, 10, 20, 0.02, 0.35).status);
        assertEquals(INVALID, clock.read(3, 10.2, 11200, 11200, Double.NaN, 20, 0.02, 0.35).status);
    }
}
