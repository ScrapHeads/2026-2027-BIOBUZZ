package org.firstinspires.ftc.teamcode.util.aim;

import com.pedropathing.math.Pose;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

/**
 * Planar Hive aiming in Pedro Pathing field coordinates (inches and radians).
 * Put this class in your TeamCode source folder and add your package declaration.
 */
public final class HiveAim {
    private static final double TURN = 2 * Math.PI;
    private static final double EPS = 1e-8;
    private static final double WIDTH = 20;
    private static final double DEPTH = 12;
    private static final double WALL = 1.2;

    private HiveAim() {}

    public enum Cell {
        // Opening center X, opening Y (inches from bottom left), and facing (+Y or -Y).
        // These field coordinates are estimates from the simulator's reference map.
        RED_NORTH(58.9333333333, 92.0, 1),
        BLUE_NORTH(85.4666666667, 92.0, 1),
        RED_SOUTH(58.9333333333, 52.0, -1),
        BLUE_SOUTH(85.4666666667, 52.0, -1);

        private final double left, right, mouth, aimY;
        private final int facing;

        Cell(double centerX, double mouthY, int facing) {
            left = centerX - WIDTH / 2;
            right = centerX + WIDTH / 2;
            mouth = mouthY;
            aimY = mouth - facing * DEPTH * 0.66;
            this.facing = facing;
        }
    }

    public enum Piece {
        POLLEN(1.4), NECTAR(1.8), POINT(0);

        private final double radius;

        Piece(double radius) {
            this.radius = radius;
        }
    }

    public static final class Solution {
        public final double aimX, aimY, turretAngle, headingTolerance;

        private Solution(double aimX, double aimY, double turretAngle, double headingTolerance) {
            this.aimX = aimX;
            this.aimY = aimY;
            this.turretAngle = turretAngle;
            this.headingTolerance = headingTolerance;
        }
    }

    /**
     * Returns an empty result when the opening is blocked or outside turret travel.
     * robotPose uses Pedro field coordinates in inches and chassis heading in radians.
     * It can come directly from RobotState.getInstance().getEstimatedPedroPose().
     * A null pose (localization not initialized) returns an empty result.
     * All turret angles and headingTolerance are in radians and chassis-relative.
     * Pass the measured turret angle to break ties between equally wide windows.
     * When enabled, min and max must be in [-pi, pi], with min < max.
     */
    public static Optional<Solution> solve(Cell cell, Piece piece,
                                           Pose robotPose, double currentTurret,
                                           boolean limitsEnabled, double min, double max) {
        if (robotPose == null) return Optional.empty();
        double robotX = robotPose.x();
        double robotY = robotPose.y();
        double robotHeading = robotPose.heading();
        if (!Double.isFinite(robotX) || !Double.isFinite(robotY)
                || !Double.isFinite(robotHeading) || !Double.isFinite(currentTurret)
                || (limitsEnabled && !(min >= -Math.PI && max <= Math.PI && min < max))
                || (robotY - cell.mouth) * cell.facing <= EPS) {
            return Optional.empty();
        }

        double dy = cell.aimY - robotY;
        double fraction = (cell.mouth - robotY) / dy;
        double left = cell.left + piece.radius;
        double right = cell.right - piece.radius;
        double lo = Math.max(left, robotX + (left - robotX) / fraction);
        double hi = Math.min(right, robotX + (right - robotX) / fraction);
        if (hi - lo < EPS) return Optional.empty();

        List<Wall> walls = new ArrayList<>();
        for (Cell other : Cell.values()) addWalls(walls, other, piece.radius);

        List<Double> cuts = new ArrayList<>();
        cuts.add(lo);
        cuts.add(hi);

        // Project wall corners onto the aim line to find visibility boundaries.
        for (Wall wall : walls) {
            double low = Math.max(wall.low, Math.min(robotY, cell.aimY));
            double high = Math.min(wall.high, Math.max(robotY, cell.aimY));
            if (low > high) continue;
            for (double y : new double[]{low, high}) {
                if (Math.abs(y - robotY) < EPS) continue;
                addCut(cuts, robotX + dy * (wall.left - robotX) / (y - robotY), lo, hi);
                addCut(cuts, robotX + dy * (wall.right - robotX) / (y - robotY), lo, hi);
            }
        }
        if (limitsEnabled) {
            for (double stop : new double[]{min, max}) {
                double bearing = robotHeading + stop;
                double scale = dy / Math.sin(bearing);
                if (scale > 0) addCut(cuts, robotX + scale * Math.cos(bearing), lo, hi);
            }
        }
        Collections.sort(cuts);

        List<Window> windows = new ArrayList<>();
        for (int i = 1; i < cuts.size(); i++) {
            double a = cuts.get(i - 1);
            double b = cuts.get(i);
            if (b - a < EPS) continue;

            double mid = (a + b) / 2;
            if (!isClear(robotX, robotY, mid, cell.aimY, walls)) continue;
            double command = toTurretAngle(Math.atan2(dy, mid - robotX) - robotHeading,
                    limitsEnabled, min, max);
            if (Double.isNaN(command)) continue;

            Window previous = windows.isEmpty() ? null : windows.get(windows.size() - 1);
            if (previous != null && Math.abs(previous.b - a) < EPS
                    && (!limitsEnabled || Math.abs(previous.command - command) < Math.PI)
                    && isClear(robotX, robotY, a, cell.aimY, walls)) {
                previous.b = b;
                previous.command = command;
            } else {
                windows.add(new Window(a, b, command));
            }
        }

        Solution best = null;
        double bestMove = Double.POSITIVE_INFINITY;
        for (Window window : windows) {
            double first = Math.atan2(dy, window.a - robotX);
            double last = Math.atan2(dy, window.b - robotX);
            double bearing = (first + last) / 2;
            double tolerance = Math.abs(last - first) / 2;
            double aimX = robotX + dy / Math.tan(bearing);

            // The final equation uses the selected point, in field coordinates.
            double angle = toTurretAngle(Math.atan2(cell.aimY - robotY, aimX - robotX)
                    - robotHeading, limitsEnabled, min, max);
            if (Double.isNaN(angle)) continue;
            double delta = angle - currentTurret;
            double move = Math.abs(limitsEnabled ? delta : wrap(delta));
            if (best == null || tolerance > best.headingTolerance + EPS
                    || (Math.abs(tolerance - best.headingTolerance) < EPS && move < bestMove)) {
                best = new Solution(aimX, cell.aimY, angle, tolerance);
                bestMove = move;
            }
        }
        return Optional.ofNullable(best);
    }

    private static double toTurretAngle(double angle, boolean limited, double min, double max) {
        double relative = wrap(angle);
        if (!limited) return relative;
        double center = (min + max) / 2;
        double equivalent = relative + TURN * Math.round((center - relative) / TURN);
        if (equivalent < min - EPS || equivalent > max + EPS) return Double.NaN;
        return Math.max(min, Math.min(max, equivalent));
    }

    private static double wrap(double angle) {
        return ((angle + Math.PI) % TURN + TURN) % TURN - Math.PI;
    }

    private static void addCut(List<Double> cuts, double x, double lo, double hi) {
        if (x > lo + EPS && x < hi - EPS) cuts.add(x);
    }

    static void addWalls(List<Wall> walls, Cell cell, double radius) {
        double back = cell.mouth - cell.facing * DEPTH;
        double low = Math.min(cell.mouth, back);
        double high = Math.max(cell.mouth, back);
        walls.add(new Wall(cell.left - WALL, cell.left, low, high, radius));
        walls.add(new Wall(cell.right, cell.right + WALL, low, high, radius));
        walls.add(new Wall(cell.left - WALL, cell.right + WALL,
                back - (cell.facing == 1 ? WALL : 0),
                back + (cell.facing == -1 ? WALL : 0), radius));
    }

    private static boolean isClear(double x1, double y1, double x2, double y2, List<Wall> walls) {
        for (Wall wall : walls) {
            if (wall.intersects(x1, y1, x2, y2)) return false;
        }
        return true;
    }

    private static final class Window {
        final double a;
        double b, command;

        Window(double a, double b, double command) {
            this.a = a;
            this.b = b;
            this.command = command;
        }
    }

    static final class Wall {
        final double left, right, low, high;

        Wall(double left, double right, double low, double high, double radius) {
            this.left = left - radius;
            this.right = right + radius;
            this.low = low - radius;
            this.high = high + radius;
        }

        boolean intersects(double x1, double y1, double x2, double y2) {
            double dx = x2 - x1;
            double dy = y2 - y1;
            double enter = 0;
            double exit = 1;
            if (Math.abs(dx) < EPS) {
                if (x1 < left || x1 > right) return false;
            } else {
                double a = (left - x1) / dx;
                double b = (right - x1) / dx;
                enter = Math.max(enter, Math.min(a, b));
                exit = Math.min(exit, Math.max(a, b));
            }
            if (Math.abs(dy) < EPS) {
                if (y1 < low || y1 > high) return false;
            } else {
                double a = (low - y1) / dy;
                double b = (high - y1) / dy;
                enter = Math.max(enter, Math.min(a, b));
                exit = Math.min(exit, Math.max(a, b));
            }
            return enter <= exit;
        }
    }
}
