package org.firstinspires.ftc.teamcode.robotTests.localization;

import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.*;
import org.firstinspires.ftc.teamcode.RilLib.Math.VecBuilder;
import org.firstinspires.ftc.teamcode.vision.localization.HiveAngleEstimator;
import org.firstinspires.ftc.teamcode.vision.localization.HiveAngleEstimator.*;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class HiveAngleEstimatorTest {
    private final HiveAngleEstimator solver = new HiveAngleEstimator();
    private final Geometry geometry = new Geometry(0.3, -0.4, 1.12, 0.6, 0);
    private Result solve(Pose3d pose, Geometry g) {
        return solver.estimate(pose, g, Math.toRadians(35), Math.toRadians(10), 0.05,
                Math.toRadians(1));
    }

    /** Independent forward model: Rodrigues rotation about the pivot's arbitrary horizontal axis. */
    private Pose3d apparent(Pose3d truth, Geometry g, double theta) {
        double phi = -theta, ux = -Math.sin(g.axisYaw), uy = Math.cos(g.axisYaw);
        double x = truth.getX() - g.pivotX, y = truth.getY() - g.pivotY;
        double z = truth.getZ() - g.pivotHeight;
        double c = Math.cos(phi), s = Math.sin(phi), dot = ux * x + uy * y;
        return new Pose3d(g.pivotX + x*c + uy*z*s + ux*dot*(1-c),
                g.pivotY + y*c - ux*z*s + uy*dot*(1-c),
                g.pivotHeight + z*c + (ux*y-uy*x)*s,
                truth.getRotation().rotateBy(new Rotation3d(VecBuilder.fill(ux, uy, 0), phi)));
    }

    @Test void recoversBothTiltDirectionsAtAllHeadingsAndOffsetPivot() {
        for (double theta : new double[]{-0.5, -0.2, 0, 0.2, 0.5}) {
            for (double yaw : new double[]{-3.13, -Math.PI/2, 0, Math.PI/2, 3.13}) {
                Pose3d truth = new Pose3d(1.25, 0.2, 0, new Rotation3d(0, 0, yaw));
                Result result = solve(apparent(truth, geometry, theta), geometry);
                assertTrue(result.isValid(), result.failure.toString());
                assertEquals(theta, result.hiveAngle, 1e-8);
                assertEquals(truth.getX(), result.correctedPose.getX(), 1e-8);
                assertEquals(truth.getY(), result.correctedPose.getY(), 1e-8);
                assertEquals(0, result.correctedPose.getZ(), 1e-8);
                assertEquals(0, HiveAngleEstimator.wrap(result.correctedPose.getRotation().getZ()
                        - yaw), 1e-8);
            }
        }
    }

    @Test void supportsRobotOriginAboveFloor() {
        Geometry g = new Geometry(0, 0, 1.12, 0, 0.1);
        Pose3d truth = new Pose3d(1, 0.2, 0.1, new Rotation3d());
        Result result = solve(apparent(truth, g, 0.3), g);
        assertTrue(result.isValid());
        assertEquals(0.1, result.correctedPose.getZ(), 1e-8);
    }

    @Test void rejectsImpossibleDegenerateAndNonfiniteInputs() {
        Geometry g = new Geometry(0, 0, 1.12, 0, 0);
        assertEquals(Failure.NO_SOLUTION,
                solve(new Pose3d(0, 0, 1.12, new Rotation3d()), g).failure);
        assertEquals(Failure.ILL_CONDITIONED,
                solve(new Pose3d(0, 0, 0, new Rotation3d()), g).failure);
        assertEquals(Failure.INVALID_INPUT,
                solve(new Pose3d(Double.NaN, 0, 0, new Rotation3d()), g).failure);
    }

    @Test void rejectsNonlevelRobotAndOutOfRangeTilt() {
        Geometry g = new Geometry(0, 0, 1.12, 0, 0);
        assertEquals(Failure.ORIENTATION_MISMATCH,
                solve(new Pose3d(1, 0, 0, new Rotation3d(0.4, 0, 0)), g).failure);
        Pose3d truth = new Pose3d(1, 0, 0, new Rotation3d());
        assertFalse(solve(apparent(truth, g, 0.8), g).isValid());
    }

    @Test void rejectsTwoNearlyEquivalentRootsInsteadOfChoosingSmallest() {
        Geometry g = new Geometry(0, 0, 1.12, 0, 0);
        Result result = solver.estimate(new Pose3d(0.005, 0, 0, new Rotation3d()), g,
                0.6, 0.2, 0.001, Math.toRadians(1));
        assertEquals(Failure.AMBIGUOUS, result.failure);
    }

    @Test void rollCheckDetectsFloorSnappingAtSidewaysHeading() {
        Geometry g = new Geometry(0, 0, 1.12, 0, 0);
        Pose3d raw = apparent(new Pose3d(1, 0, 0,
                new Rotation3d(0, 0, Math.PI/2)), g, 0.4);
        Pose3d snapped = new Pose3d(raw.getX(), raw.getY(), 0, raw.getRotation());
        assertFalse(solve(snapped, g).isValid());
    }
}
