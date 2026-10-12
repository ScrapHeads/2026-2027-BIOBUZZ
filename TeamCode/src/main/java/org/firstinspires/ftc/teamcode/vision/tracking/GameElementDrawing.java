package org.firstinspires.ftc.teamcode.vision.tracking;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.pedropathing.math.Pose;


public class GameElementDrawing {

    // Dimensions in inches (replace with measured dimensions)
    private static final double POLLEN_RADIUS = 1.4;
    private static final double NECTAR_RADIUS = 1.8;

    // FTC Dashboard colors
    private static final String POLLEN_COLOR = "#FFD700";
    private static final String BLUE_NECTAR_COLOR = "#2196F3";
    private static final String RED_NECTAR_COLOR = "#F44336";

    private static final int STROKE_WIDTH = 1;

    public static void drawPollen(Canvas c, Pose pose) {
        drawElement(c, pose, POLLEN_RADIUS, POLLEN_COLOR);
    }

    public static void drawBlueNectar(Canvas c, Pose pose) {
        drawElement(c, pose, NECTAR_RADIUS, BLUE_NECTAR_COLOR);
    }

    public static void drawRedNectar(Canvas c, Pose pose) {
        drawElement(c, pose, NECTAR_RADIUS, RED_NECTAR_COLOR);
    }

    private static void drawElement(
            Canvas c,
            Pose pose,
            double radius,
            String color
    ) {
        c.setStroke(color);
        c.setStrokeWidth(STROKE_WIDTH);
        c.strokeCircle(
                pose.x(),
                pose.y(),
                radius
        );
    }
}