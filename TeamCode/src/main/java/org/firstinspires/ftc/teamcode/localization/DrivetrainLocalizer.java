package org.firstinspires.ftc.teamcode.localization;


import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.math.Vector;
import com.pedropathing.math.Velocity;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.util.ConversionUtil;
import org.firstinspires.ftc.teamcode.util.TimeTracker;

import java.util.Map;

/**
 * Custom Localizer that integrates GoBilda Pinpoint and Limelight Vision via RobotState.
 * This class acts as the bridge between the hardware and Pedro Pathing.
 */
public class DrivetrainLocalizer implements Localizer {

    private final PinpointLocalizer pinpoint;
    private Pose pedroPose = new Pose(0, 0, 0);

    public DrivetrainLocalizer (HardwareMap hardwareMap, PinpointConfig pinpointConstants) {
        pinpoint = new PinpointLocalizer(hardwareMap, pinpointConstants);
    }

    @Override
    public void setPose(Pose setPose) {
        pinpoint.setPose(setPose);
        RobotState.getInstance().addOdometryObservation(ConversionUtil.toRil(setPose), TimeTracker.getTime());
    }

    @Override
    public void setX(double x) {
        pinpoint.setX(x);
    }

    @Override
    public void setY(double y) {
        pinpoint.setY(y);
    }

    @Override
    public void setHeading(double heading) {
        pinpoint.setHeading(heading);
    }

    @Override
    public Pose pose() {
        return pinpoint.pose();
    }

    @Override
    public Twist twist() {
        return pinpoint.twist();
    }

    @Override
    public Velocity velocity() {
        return pinpoint.velocity();
    }

    @Override
    public MotionState state() {
        return null;
    }

    @Override
    public void update() {
        // 1. Update the Pinpoint hardware
        pinpoint.update();

        // 2. Push the raw Pinpoint pose (Odometry) to the centralized PoseEstimator in RobotState
        // We use TimeTracker to ensure timestamps are consistent with vision data
        RobotState.getInstance().addOdometryObservation(
                ConversionUtil.toRil(pinpoint.pose()),
                TimeTracker.getTime()
        );

        // 3. Fetch the fused estimate (Odometry + Vision) back from RobotState for Pedro Pathing
        pedroPose = ConversionUtil.toPedro(RobotState.getInstance().getEstimatedPose());
    }

    @Override
    public void reset() {

    }

    @Override
    public Map<String, Object> debug() {
        return Localizer.super.debug();
    }
}
