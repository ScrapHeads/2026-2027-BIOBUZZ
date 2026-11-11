package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.TurretPosConfig.*;
import static org.firstinspires.ftc.teamcode.util.PIDUtil.updatePIDController;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.RilLib.Math.Units;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.config.TurretPosConfig;
import org.firstinspires.ftc.teamcode.util.aim.AimSolution;
import org.firstinspires.ftc.teamcode.util.aim.HiveAim;
import org.firstinspires.ftc.teamcode.util.aim.HiveAimDashboard;

import java.util.Optional;

public class TurretRotSubsystem implements Subsystem {
    // TODO: Set encoder ticks per motor output-shaft revolution, including the motor's gearbox.
    public static final double MOTOR_TPR = 28;
    // TODO: Set turret revolutions per motor output-shaft revolution (1/3 means 3:1 reduction).
    public static final double GEAR_RATIO = 1.0 / 3.0;
    // TODO: Set the turret's known physical angle at initialization; the encoder resets there.
    public static final double START_ANGLE_DEGREES = 0.0;

    public static final double TICKS_PER_REV = MOTOR_TPR / GEAR_RATIO;

    private final MotorEx turretRot;
    private final PIDFController controller;
    private final MultipleTelemetry telemetry;

    public TurretRotSubsystem(HardwareMap hm, MultipleTelemetry telemetry) {
        turretRot = new MotorEx(hm, "turretRot");
        this.telemetry = telemetry;

        turretRot.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        turretRot.encoder.reset();

        controller = new PIDFController(
                pidfValues.kP,
                pidfValues.kI,
                pidfValues.kD,
                pidfValues.kF
        );
        pidfValues.setLasts();
    }

    public void setPower(double power) {
        turretRot.set(Math.max(-1.0, Math.min(1.0, power)));
    }

    public double getPower() {
        return turretRot.get();
    }

    public double getTicksPerSec() {
        return turretRot.encoder.getRawVelocity();
    }

    /** Returns the unwrapped turret angle in degrees, relative to the startup reference. */
    public double getAngleDegrees() {
        return START_ANGLE_DEGREES + ((turretRot.encoder.getPosition() * 360.0) / TICKS_PER_REV);
    }

    /** Sets a signed angle in degrees and enables PID to turn to and hold that angle. */
    public void turnToAngle(double angleDegrees) {
        targetAngleDegrees = angleDegrees;
        setEnablePID(true);
    }

    public double getTargetAngleDegrees() {
        return targetAngleDegrees;
    }

    public void setEnablePID(boolean enablePID) {
        TurretPosConfig.enablePID = enablePID;
    }

    public boolean getEnablePID() {
        return enablePID;
    }

    private void sendTelemetry() {
        telemetry.addData("Current Angle (deg): ", getAngleDegrees());
        telemetry.addData("Target Angle (deg): ", getTargetAngleDegrees());
        telemetry.addData("Is PID controller on: ", getEnablePID());
    }

    @Override
    public void periodic() {
        updatePIDController(controller, pidfValues);

        RobotState robotState = RobotState.getInstance();
        Optional<AimSolution> sharedAim = robotState.getAimSolution();
        Pose pose = sharedAim.map(aim -> new Pose(aim.robotXInches, aim.robotYInches, 0.0))
                .orElseGet(robotState::getEstimatedPedroPose);
        HiveAim.Cell cell = robotState.getTargetCell();
        double currentAngleDegrees = getAngleDegrees();
        Optional<HiveAim.Solution> solution = sharedAim.map(aim -> aim.solution);

        if (enablePID) {
            if (solveAngle) {
                targetAngleDegrees = getPosToAngle(solution);
            }
            double pidOut = controller.calculate(currentAngleDegrees, targetAngleDegrees);
            setPower(pidOut);
        } else {
            setPower(targetPower);
        }

        sendTelemetry();
        if (showAimDashboard) {
            sendAimDashboard(pose, cell, solution, currentAngleDegrees);
        }
    }

    /** Returns the solved turret angle in degrees, or the existing target if no shot is available. */
    private double getPosToAngle(Optional<HiveAim.Solution> solution) {
        return solution.map(aim -> Units.radiansToDegrees(aim.turretAngle))
                .orElse(getTargetAngleDegrees());
    }

    private void sendAimDashboard(Pose pose, HiveAim.Cell cell,
                                  Optional<HiveAim.Solution> solution, double currentAngleDegrees) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Turret/Cell", cell == null ? "Alliance not set" : cell.name());
        packet.put("Turret/Pose available", pose != null);
        packet.put("Turret/Solution available", solution.isPresent());
        packet.put("Turret/Current angle (deg)", currentAngleDegrees);
        packet.put("Turret/Target angle (deg)", getTargetAngleDegrees());
        packet.put("Turret/PID enabled", enablePID);
        packet.put("Turret/Auto aim enabled", solveAngle);
        if (solution.isPresent()) {
            HiveAim.Solution aim = solution.get();
            packet.put("Turret/Solved angle (deg)", Units.radiansToDegrees(aim.turretAngle));
            packet.put("Turret/Heading tolerance (deg)", Units.radiansToDegrees(aim.headingTolerance));
        }

        // With no pose, draw just the hive outlines; no aiming line is available.
        HiveAimDashboard.draw(packet.fieldOverlay(),
                pose == null ? 0.0 : pose.x(),
                pose == null ? 0.0 : pose.y(), solution);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
