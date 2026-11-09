package org.firstinspires.ftc.teamcode.vision.localization;

import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose3d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation3d;

/**
 * Recovers the robot pose when MT1 used a flat map but the observed HIVE was tilted.
 *
 * <p>The apparent robot pose is the true pose rotated about the HIVE pivot by
 * phi = -theta, where theta is the actual HIVE angle. The known height of the
 * robot's pose origin supplies an equation for phi. Each candidate is then
 * inverted and checked against the assumption that the robot is level.
 *
 * <p>All distances are meters and all angles are radians. Input and output poses
 * use the reference map frame; the caller applies any map-to-team-field transform.
 * All tags contributing to the input pose must belong to the same rigid HIVE.
 */
public final class HiveAngleEstimator {
    public enum Failure { NONE, INVALID_INPUT, NO_SOLUTION, ANGLE_LIMIT,
        ORIENTATION_MISMATCH, AMBIGUOUS, ILL_CONDITIONED }

    public static final class Geometry {
        // Pivot location in the reference map, horizontal axis direction, and the
        // known map-frame Z of the robot pose origin (usually 0, NOT camera height).
        public final double pivotX, pivotY, pivotHeight, axisYaw, robotHeight;
        public Geometry(double x, double y, double height, double axisYaw, double robotHeight) {
            this.pivotX = x;
            this.pivotY = y;
            this.pivotHeight = height;
            // Local +Y is the pivot axis, rotated by axisYaw about map +Z.
            this.axisYaw = axisYaw;
            this.robotHeight = robotHeight;
        }
    }

    public static final class Result {
        public final Failure failure;
        public final Pose3d correctedPose;
        // hiveAngle = theta = -phi.
        // orientationError: angle between corrected robot up and field up.
        // sensitivity: absolute height-equation derivative, in meters per radian;
        // larger values make height noise less damaging to the solved angle.
        public final double hiveAngle, orientationError, sensitivity;
        private Result(Failure failure, Pose3d pose, double angle, double error, double sensitivity) {
            this.failure = failure;
            this.correctedPose = pose;
            this.hiveAngle = angle;
            this.orientationError = error;
            this.sensitivity = sensitivity;
        }
        public boolean isValid() { return failure == Failure.NONE; }
        private static Result fail(Failure reason) {
            return new Result(reason, null, Double.NaN, Double.NaN, Double.NaN);
        }
    }

    /**
     * @param raw full apparent MT1 robot pose from the static flat-HIVE map
     * @param g measured pivot geometry and known robot-origin height
     * @param maxAngle largest physically accepted absolute HIVE tilt
     * @param maxOrientationError maximum corrected robot tilt away from level
     * @param minSensitivity minimum height change per radian for a trustworthy root
     * @param ambiguityMargin required separation between the two orientation errors
     */
    public Result estimate(Pose3d raw, Geometry g, double maxAngle,
                           double maxOrientationError, double minSensitivity,
                           double ambiguityMargin) {
        if (raw == null || g == null || !finite(raw.getX(), raw.getY(), raw.getZ(),
                raw.getRotation().getX(), raw.getRotation().getY(), raw.getRotation().getZ(),
                g.pivotX, g.pivotY, g.pivotHeight, g.axisYaw, g.robotHeight,
                maxAngle, maxOrientationError, minSensitivity, ambiguityMargin)
                || g.pivotHeight <= g.robotHeight || maxAngle <= 0 || maxAngle >= Math.PI / 2
                || maxOrientationError <= 0 || minSensitivity <= 0 || ambiguityMargin < 0) {
            return Result.fail(Failure.INVALID_INPUT);
        }
        // 1. Shift the horizontal origin to the pivot and rotate coordinates by
        // -axisYaw. The pivot axis is now local +Y, so tilt affects only local X/Z.
        // This is a change of coordinates, not a physical rotation of the robot.
        double c = Math.cos(g.axisYaw), s = Math.sin(g.axisYaw);
        double dx = raw.getX() - g.pivotX, dy = raw.getY() - g.pivotY;
        double x = c * dx + s * dy, y = -s * dx + c * dy;
        // 2. Undoing the apparent rotation must place the robot origin at its
        // known height:
        //   correctedZ = x*sin(phi) + (rawZ - h)*cos(phi) + h = robotHeight
        // Rearrange to A*sin(phi) + B*cos(phi) = C, with A=x, B=b, C=target.
        double b = raw.getZ() - g.pivotHeight;
        double target = g.robotHeight - g.pivotHeight;
        // The left side cannot exceed hypot(A, B) in magnitude. If |C| exceeds
        // that amplitude, no rotation can satisfy the assumed robot height.
        double radius = Math.hypot(x, b);
        if (radius < 1e-12 || Math.abs(target) > radius + 1e-10) {
            return Result.fail(Failure.NO_SOLUTION);
        }
        // 3. Write A*sin(phi) + B*cos(phi) as radius*sin(phi + beta).
        // sin(u)=C/radius has two solutions: u=alpha and u=pi-alpha.
        // Clamp only after the solvability check to absorb floating-point roundoff.
        double alpha = Math.asin(Math.max(-1, Math.min(1, target / radius)));
        double beta = Math.atan2(b, x);
        double[] roots = {wrap(alpha - beta), wrap(Math.PI - alpha - beta)};
        Result best = null, second = null;
        boolean inRange = false, oriented = false;
        // Express orientation in the same local pivot-axis frame as position.
        // RilLib rotateBy applies its argument about the fixed/global axes:
        // localRaw = Rz(-axisYaw) * rawRotation.
        Rotation3d localRaw = raw.getRotation().rotateBy(new Rotation3d(0, 0, -g.axisYaw));
        for (int i = 0; i < roots.length; i++) {
            // 4. Evaluate both roots. A repeated root is only one candidate;
            // candidates outside the mechanical window cannot describe the HIVE.
            double phi = roots[i];
            if (i == 1 && Math.abs(wrap(phi - roots[0])) < 1e-8) continue;
            if (Math.abs(phi) > maxAngle + 1e-10) continue;
            inRange = true;
            // Invert the apparent HIVE-induced orientation:
            // correctedRotation = Ry(-phi) * localRaw.
            Rotation3d localCorrected = localRaw.rotateBy(new Rotation3d(0, -phi, 0));
            // For R=Rz(yaw)*Ry(pitch)*Rx(roll), R[2,2]=cos(pitch)*cos(roll).
            // This is the dot product of robot up with field up. acos gives the
            // total departure from level, checking roll as well as pitch even
            // when the robot faces sideways relative to the HIVE tilt plane.
            double error = Math.acos(Math.max(-1, Math.min(1,
                    Math.cos(localCorrected.getX()) * Math.cos(localCorrected.getY()))));
            if (error > maxOrientationError) continue;
            oriented = true;
            // 5. Differentiate the height constraint:
            // dZ/dphi = x*cos(phi) - b*sin(phi).
            // Near zero, a tiny height error can require a large angle change
            // (approximately |angleError| = |heightError| / sensitivity).
            // Reject that ill-conditioned geometry instead of trusting a noisy root.
            double sensitivity = Math.abs(x * Math.cos(phi) - b * Math.sin(phi));
            if (sensitivity < minSensitivity) continue;
            // 6. Rotate the apparent X/Z offset about the pivot by -phi.
            // Local Y is unchanged because it lies along the rotation axis.
            // Corrected Z equals robotHeight by construction; it is NOT an
            // independent quality check. Orientation above supplies that check.
            double correctedX = x * Math.cos(phi) - b * Math.sin(phi);
            double correctedZ = x * Math.sin(phi) + b * Math.cos(phi) + g.pivotHeight;
            // Return from local pivot coordinates to the reference map:
            // rotate horizontal position/orientation by +axisYaw, then restore
            // the pivot's horizontal translation. correctedZ is already map Z.
            Pose3d pose = new Pose3d(g.pivotX + c * correctedX - s * y,
                    g.pivotY + s * correctedX + c * y, correctedZ,
                    localCorrected.rotateBy(new Rotation3d(0, 0, g.axisYaw)));
            // The physical HIVE angle has the opposite sign from the apparent
            // pose rotation. Rank surviving roots by how level they make the robot.
            Result candidate = new Result(Failure.NONE, pose, -phi, error, sensitivity);
            if (best == null || error < best.orientationError) {
                second = best;
                best = candidate;
            } else second = candidate;
        }
        if (best == null) return Result.fail(!inRange ? Failure.ANGLE_LIMIT
                : !oriented ? Failure.ORIENTATION_MISMATCH : Failure.ILL_CONDITIONED);
        // 7. If both roots fit almost equally well, the observation does not
        // distinguish them reliably. Reject it rather than bias toward zero tilt.
        if (second != null && second.orientationError - best.orientationError <= ambiguityMargin) {
            return Result.fail(Failure.AMBIGUOUS);
        }
        return best;
    }

    /** Equivalent angle in [-pi, pi], so roots across the +/-pi boundary compare correctly. */
    public static double wrap(double angle) { return Math.atan2(Math.sin(angle), Math.cos(angle)); }
    public static boolean finite(double... values) {
        for (double value : values) if (!Double.isFinite(value)) return false;
        return true;
    }
}
