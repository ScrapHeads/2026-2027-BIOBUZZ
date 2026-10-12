package org.firstinspires.ftc.teamcode.robotTests.visionTests;

import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.vision.tracking.GameElementType;
import org.firstinspires.ftc.teamcode.vision.tracking.TrackedGameElement;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

public class TrackedGameElementTest {
    @Test
    @DisplayName("Create Tracked Game Element Test")
    public void testCreateTrackedGameElement () {
        Pose2d pose = new Pose2d(1,1, new Rotation2d(2));
        double id = 1100;
        GameElementType element = TrackedGameElement.calcElementType(id);
        double confidence = .5;
        double lastSeenTime = 100;
        TrackedGameElement trackedGameElement = new TrackedGameElement(id, element, pose, confidence, lastSeenTime);
    }
}
