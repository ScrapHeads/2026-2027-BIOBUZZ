package org.firstinspires.ftc.teamcode.util;

/** Monotonic seconds shared by localization only; never reset between OpModes. */
public final class LocalizationClock {
    private LocalizationClock() {}
    public static double now() { return System.nanoTime() * 1e-9; }
}
