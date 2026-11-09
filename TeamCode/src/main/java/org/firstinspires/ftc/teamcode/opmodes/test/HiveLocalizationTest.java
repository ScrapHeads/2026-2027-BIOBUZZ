package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.util.ConversionUtil;
import org.firstinspires.ftc.teamcode.util.LocalizationClock;
import org.firstinspires.ftc.teamcode.vision.localization.LimelightLocalizer;
import org.firstinspires.ftc.teamcode.config.pedro.Constants;
import org.firstinspires.ftc.teamcode.localization.DrivetrainLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/** End-to-end localization validation using manual motion; no drivetrain motors initialized. */
@Config
@TeleOp(name = "TEST - Hive Localization", group = "Test")
public class HiveLocalizationTest extends LinearOpMode {
    public static boolean BLUE = false;
    public static double START_X_IN = 0, START_Y_IN = 0, START_HEADING_DEG = 0;
    // Match GameElementMap's corner-origin Pedro -> centered Dashboard translation.
    // Set both to zero if localization is configured with a field-centered origin.
    public static double FIELD_OFFSET_X_IN = -72, FIELD_OFFSET_Y_IN = -72;
    public static double VISION_MAX_DRAW_AGE_SECONDS = 0.5;
    private static final String ODOMETRY_COLOR = "#FF9800";
    private static final String VISION_COLOR = "#AB47BC";
    private static final String FUSED_COLOR = "#00C853";

    @Override public void runOpMode() throws InterruptedException {
        MultipleTelemetry output = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DrivetrainLocalizer drive = new DrivetrainLocalizer(hardwareMap, Constants.pinpointLocalizerConfig);
        drive.setPose(startPose());
        RobotState.getInstance().setTeam(BLUE);
        try (VisionSubsystem vision = new VisionSubsystem(hardwareMap, output, FtcDashboard.getInstance())) {
            drive.setVisionUpdate(vision::updateLocalization);
            output.addLine("Set surveyed START pose and HIVE config before INIT.");
            output.addLine("Move robot by hand. A resets to START; no drive motors are initialized.");
            output.update();
            waitForStart();
            boolean previousA = false;
            double lastResetTime = LocalizationClock.now();
            while (opModeIsActive()) {
                boolean pressed = gamepad1.a;
                if (pressed && !previousA) {
                    drive.setPose(startPose());
                    lastResetTime = LocalizationClock.now();
                }
                previousA = pressed;
                vision.periodic();
                drive.update();
                output.addData("Raw odometry (m/rad)", RobotState.getInstance().getOdometryPose());
                output.addData("Fused pose (m/rad)", RobotState.getInstance().getEstimatedPose());
                output.addData("Pedro pose (in/rad)", drive.pose());
                output.addData("Field colors", "Orange: Pinpoint | Purple: accepted vision | Green: fused/Pedro");
                output.update();
                drawField(vision.getLocalizer(), lastResetTime);
                sleep(20);
            }
        }
    }

    private void drawField(LimelightLocalizer vision, double lastResetTime) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Field orange", "Raw Pinpoint odometry");
        packet.put("Field purple", "Last accepted corrected vision (capture-time pose)");
        packet.put("Field green", "Fused pose / Pedro");
        Canvas canvas = packet.fieldOverlay();
        canvas.setTranslation(FIELD_OFFSET_X_IN, FIELD_OFFSET_Y_IN);
        // Different radii keep coincident estimates visible as concentric rings.
        drawPose(canvas, RobotState.getInstance().getOdometryPose(), ODOMETRY_COLOR, 9);
        double age = vision == null ? Double.NaN
                : LocalizationClock.now() - vision.getLastAcceptedTime();
        boolean showVision = vision != null && vision.getLastAcceptedTime() >= lastResetTime
                && Double.isFinite(age) && age >= 0 && age <= VISION_MAX_DRAW_AGE_SECONDS;
        if (showVision) drawPose(canvas, vision.getLastAcceptedPose(), VISION_COLOR, 7);
        packet.put("Field vision visible", showVision);
        if (Double.isFinite(age)) packet.put("Field vision age (s)", age);
        drawPose(canvas, RobotState.getInstance().getEstimatedPose(), FUSED_COLOR, 5);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    private void drawPose(Canvas canvas, Pose2d pose, String color, double radiusInches) {
        if (!RobotState.validPose(pose)) return;
        Pose drawingPose = ConversionUtil.toPedro(pose);
        double x = drawingPose.x(), y = drawingPose.y(), heading = drawingPose.heading();
        canvas.setStroke(color);
        canvas.setStrokeWidth(2);
        canvas.strokeCircle(x, y, radiusInches);
        canvas.strokeLine(x, y, x + radiusInches * Math.cos(heading),
                y + radiusInches * Math.sin(heading));
    }

    private Pose startPose() {
        return new Pose(START_X_IN, START_Y_IN, Math.toRadians(START_HEADING_DEG));
    }
}
