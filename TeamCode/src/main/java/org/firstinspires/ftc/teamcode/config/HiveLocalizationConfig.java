package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.vision.localization.HiveAngleEstimator.Geometry;

/** Configure before INIT; restart the OpMode after edits. Map and geometry are per HIVE. */
@Config
public final class HiveLocalizationConfig {
    private HiveLocalizationConfig() {}
    public static boolean ENABLED = true;
    // TODO: Verify each pipeline really uses its corresponding static flat-HIVE map.
    public static int RED_PIPELINE = 0, BLUE_PIPELINE = 1;
    // TODO: Replace Blue Power reference geometry with measured ScrapHeads field geometry.
    public static double RED_PIVOT_X_M = 0, RED_PIVOT_Y_M = 0;
    public static double BLUE_PIVOT_X_M = 0, BLUE_PIVOT_Y_M = 0;
    public static double PIVOT_HEIGHT_M = 43.95 * 0.0254;
    public static double RED_AXIS_YAW_RAD = 0, BLUE_AXIS_YAW_RAD = 0;
    public static double ROBOT_ORIGIN_HEIGHT_M = 0;
    // TODO: Verify map -> existing Pedro field origin/axes; extrinsics belong in Limelight UI.
    public static double RED_FIELD_X_M = 0, RED_FIELD_Y_M = 0, RED_FIELD_YAW_RAD = 0;
    public static double BLUE_FIELD_X_M = 0, BLUE_FIELD_Y_M = 0, BLUE_FIELD_YAW_RAD = 0;
    // TODO: Measure mechanical tilt limits and tune these gates using the diagnostic TeleOps.
    public static double MAX_TILT_DEG = 35, MAX_LEVEL_ERROR_DEG = 10;
    public static double MIN_SENSITIVITY_M = 0.05, ROOT_MARGIN_DEG = 1;
    public static int MIN_TAG_COUNT = 1;
    public static double MAX_DISTANCE_M = 4, MAX_AGE_SECONDS = 0.35;
    public static double MAX_TRANSLATION_ERROR_M = 0.75, MAX_HEADING_ERROR_DEG = 35;
    // TODO: Set bounds to measured field limits in the team's canonical frame.
    public static double MIN_FIELD_X_M = -2.2, MAX_FIELD_X_M = 2.2;
    public static double MIN_FIELD_Y_M = -2.2, MAX_FIELD_Y_M = 2.2;
    // TODO: Measure USB/polling transport delay. SDK capture + targeting latency omit it.
    public static double TRANSPORT_DELAY_SECONDS = 0.02;
    public static double POSITION_STD_M = 0.18, HEADING_STD_RAD = 0.35;
    public static double MAX_POSITION_STD_M = 1.5, MAX_HEADING_STD_RAD = 1.2;

    public static final class Parameters {
        public final boolean blue;
        public final int pipeline, firstTag, lastTag, minTags;
        public final Geometry geometry;
        public final double fieldX, fieldY, fieldYaw, maxTilt, maxLevelError, minSensitivity,
                rootMargin, maxDistance, maxAge, maxTranslationError, maxHeadingError,
                minX, maxX, minY, maxY, transportDelay, positionStd, headingStd,
                maxPositionStd, maxHeadingStd;
        public Parameters(boolean blue) {
            this.blue = blue;
            pipeline = blue ? BLUE_PIPELINE : RED_PIPELINE;
            firstTag = blue ? 38 : 30;
            lastTag = blue ? 45 : 37;
            geometry = new Geometry(blue ? BLUE_PIVOT_X_M : RED_PIVOT_X_M,
                    blue ? BLUE_PIVOT_Y_M : RED_PIVOT_Y_M, PIVOT_HEIGHT_M,
                    blue ? BLUE_AXIS_YAW_RAD : RED_AXIS_YAW_RAD, ROBOT_ORIGIN_HEIGHT_M);
            fieldX = blue ? BLUE_FIELD_X_M : RED_FIELD_X_M;
            fieldY = blue ? BLUE_FIELD_Y_M : RED_FIELD_Y_M;
            fieldYaw = blue ? BLUE_FIELD_YAW_RAD : RED_FIELD_YAW_RAD;
            minTags = MIN_TAG_COUNT;
            maxTilt = Math.toRadians(MAX_TILT_DEG);
            maxLevelError = Math.toRadians(MAX_LEVEL_ERROR_DEG);
            minSensitivity = MIN_SENSITIVITY_M;
            rootMargin = Math.toRadians(ROOT_MARGIN_DEG);
            maxDistance = MAX_DISTANCE_M;
            maxAge = MAX_AGE_SECONDS;
            maxTranslationError = MAX_TRANSLATION_ERROR_M;
            maxHeadingError = Math.toRadians(MAX_HEADING_ERROR_DEG);
            minX = MIN_FIELD_X_M; maxX = MAX_FIELD_X_M;
            minY = MIN_FIELD_Y_M; maxY = MAX_FIELD_Y_M;
            transportDelay = TRANSPORT_DELAY_SECONDS;
            positionStd = POSITION_STD_M; headingStd = HEADING_STD_RAD;
            maxPositionStd = MAX_POSITION_STD_M; maxHeadingStd = MAX_HEADING_STD_RAD;
        }
    }
}
