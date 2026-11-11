package org.firstinspires.ftc.teamcode.util.aim;

import com.acmerobotics.dashboard.canvas.Canvas;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Optional;

/** Draws the Hive model on FTC Dashboard. Use the same package as HiveAim. */
public final class HiveAimDashboard {
    private static final double FIELD_CENTER = 72.0;
    private static final EnumMap<HiveAim.Cell, List<HiveAim.Wall>> WALLS =
            new EnumMap<>(HiveAim.Cell.class);

    static {
        for (HiveAim.Cell cell : HiveAim.Cell.values()) {
            List<HiveAim.Wall> walls = new ArrayList<>();
            HiveAim.addWalls(walls, cell, 0.0);
            WALLS.put(cell, walls);
        }
    }

    private HiveAimDashboard() {}

    /**
     * Draws all four goal outlines and, when present, the line to the solved aim point.
     * Pass Pedro Pathing field coordinates in inches and the solution from the same pose.
     * Use a field overlay in FTC Dashboard's default field frame.
     */
    public static void draw(Canvas overlay, double robotX, double robotY,
            Optional<HiveAim.Solution> solution) {
        overlay.setStrokeWidth(1);
        for (HiveAim.Cell cell : HiveAim.Cell.values()) {
            boolean red = cell == HiveAim.Cell.RED_NORTH || cell == HiveAim.Cell.RED_SOUTH;
            overlay.setStroke(red ? "#ef4444" : "#3b82f6");
            for (HiveAim.Wall wall : WALLS.get(cell)) {
                // Rotating the rectangle into the FTC frame swaps width and height.
                overlay.strokeRect(dashboardX(wall.high), dashboardY(wall.left),
                        wall.high - wall.low, wall.right - wall.left);
            }
        }

        if (solution.isPresent()) {
            HiveAim.Solution aim = solution.get();
            double aimX = dashboardX(aim.aimY);
            double aimY = dashboardY(aim.aimX);
            overlay.setStroke("#22c55e").setStrokeWidth(2);
            overlay.strokeLine(dashboardX(robotY), dashboardY(robotX), aimX, aimY);
            overlay.strokeCircle(aimX, aimY, 1.0);
        }
    }

    // Inverse of FTC-to-Pedro: pedroX = ftcY + 72, pedroY = 72 - ftcX.
    private static double dashboardX(double pedroY) {
        return FIELD_CENTER - pedroY;
    }

    private static double dashboardY(double pedroX) {
        return pedroX - FIELD_CENTER;
    }
}
