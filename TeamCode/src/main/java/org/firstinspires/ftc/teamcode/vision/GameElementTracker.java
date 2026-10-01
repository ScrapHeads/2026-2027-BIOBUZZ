package org.firstinspires.ftc.teamcode.vision;

import java.util.ArrayList;
import java.util.List;

public class GameElementTracker {

    private static final double MERGE_DISTANCE_CM = 0.635; // 0.25 inch

    private final List<TrackedGameElement> elements =
            new ArrayList<>();

    public void addDetection(
            GameElementType type,
            double fieldX,
            double fieldY
    ) {

        TrackedGameElement nearest = null;
        double nearestDistance = Double.MAX_VALUE;

        for (TrackedGameElement element : elements) {

            if (element.getType() != type) {
                continue;
            }

            double distance =
                    element.distanceTo(fieldX, fieldY);

            if (distance < nearestDistance) {
                nearestDistance = distance;
                nearest = element;
            }
        }

        if (nearest != null &&
                nearestDistance <= MERGE_DISTANCE_CM) {

            nearest.updatePosition(fieldX, fieldY);

        } else {

            elements.add(
                    new TrackedGameElement(
                            type,
                            fieldX,
                            fieldY
                    )
            );
        }
    }
}