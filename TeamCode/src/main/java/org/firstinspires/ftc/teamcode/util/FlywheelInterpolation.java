package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.config.FlywheelConfig.FLYWHEEL_TABLE;

import org.firstinspires.ftc.teamcode.RilLib.Math.Interpolation.InterpolatingTreeMap;

public class FlywheelInterpolation {

    private final InterpolatingTreeMap<Double, Double> rpmMap =
            new InterpolatingTreeMap<>(
                    (start, end, value) ->
                            (value - start) / (end - start),
                    (start, end, t) ->
                            start + (end - start) * t
            );

    public double getRPM (double distanceCm) {
        Double rpm = rpmMap.get(distanceCm);
        return rpm != null ? rpm : -1;
    }

    public double getRPMRefresh(double distanceCm) {
        rpmMap.clear();

        for (double[] point : FLYWHEEL_TABLE) {
            rpmMap.put(point[0], point[1]);
        }

        Double rpm = rpmMap.get(distanceCm);
        return rpm != null ? rpm : -1;
    }
}