package org.firstinspires.ftc.teamcode.localization;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.math.Velocity;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.RilLib.Math.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.util.ConversionUtil;
import org.firstinspires.ftc.teamcode.util.LocalizationClock;

import java.util.function.DoubleConsumer;

/** One raw odometry update, then cached vision correction, then one fused Pedro state. */
public class DrivetrainLocalizer implements Localizer {
    private final PinpointLocalizer pinpoint;
    private final RobotState robotState = RobotState.getInstance();
    private Pose pedroPose;
    private long poseGeneration;
    private DoubleConsumer visionUpdate = time -> {};

    public DrivetrainLocalizer(HardwareMap hardwareMap, PinpointConfig config) {
        pinpoint = new PinpointLocalizer(hardwareMap, config);
        setPose(pinpoint.pose());
    }

    public void setVisionUpdate(DoubleConsumer update) {
        visionUpdate = update == null ? time -> {} : update;
    }

    @Override public void setPose(Pose pose) {
        pinpoint.setPose(pose);
        robotState.resetPose(ConversionUtil.toRil(pose), LocalizationClock.now());
        poseGeneration = robotState.getPoseGeneration();
        pedroPose = pose;
    }
    @Override public void setX(double x) { setPose(new Pose(x, pedroPose.y(), pedroPose.heading())); }
    @Override public void setY(double y) { setPose(new Pose(pedroPose.x(), y, pedroPose.heading())); }
    @Override public void setHeading(double heading) { setPose(new Pose(pedroPose.x(), pedroPose.y(), heading)); }
    @Override public Pose pose() { return pedroPose; }

    @Override public Velocity velocity() {
        // Rotate the raw field velocity by the fused heading correction. Do not
        // differentiate pose jumps from vision to manufacture robot motion.
        Velocity raw = pinpoint.velocity();
        double offset = pedroPose.heading() - pinpoint.pose().heading();
        double c = Math.cos(offset), s = Math.sin(offset);
        return new Velocity(c * raw.vx - s * raw.vy, s * raw.vx + c * raw.vy, raw.omega);
    }
    @Override public Twist twist() { return state().twist(); }
    @Override public MotionState state() { return MotionState.ofVelocity(pedroPose, velocity()); }

    @Override public void update() {
        // External reset/handoff changes both hardware origin and fusion history.
        if (poseGeneration != robotState.getPoseGeneration()) {
            pedroPose = ConversionUtil.toPedro(robotState.getEstimatedPose());
            pinpoint.setPose(pedroPose);
            poseGeneration = robotState.getPoseGeneration();
        }
        pinpoint.update();
        double now = LocalizationClock.now();
        robotState.addOdometryObservation(ConversionUtil.toRil(pinpoint.pose()), now);
        visionUpdate.accept(now);
        pedroPose = ConversionUtil.toPedro(robotState.getEstimatedPose());
        Velocity v = velocity();
        robotState.setChassisSpeeds(new ChassisSpeeds(v.vx * 0.0254, v.vy * 0.0254, v.omega));
    }

    @Override public void reset() {
        pinpoint.reset();
        pinpoint.update();
        setPose(pinpoint.pose());
    }
}
