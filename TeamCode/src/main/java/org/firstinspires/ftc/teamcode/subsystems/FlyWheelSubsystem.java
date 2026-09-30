package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.FlywheelConfig.enablePID;
import static org.firstinspires.ftc.teamcode.config.FlywheelConfig.pidfValues;
import static org.firstinspires.ftc.teamcode.config.FlywheelConfig.targetPower;
import static org.firstinspires.ftc.teamcode.config.FlywheelConfig.targetRPM;
import static org.firstinspires.ftc.teamcode.util.PIDUtil.updatePIDController;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.config.FlywheelConfig;

public class FlyWheelSubsystem implements Subsystem {
    private static MotorEx flywheel;
    private final PIDFController controller;
    private final MultipleTelemetry telemetry;

    public static final double MOTOR_TPR = 1; //todo

    public static final double GEAR_RATIO = 1.0/3.0; //todo

    public static final double TICKS_PER_REV = MOTOR_TPR / GEAR_RATIO;

    public FlyWheelSubsystem(HardwareMap hm, MultipleTelemetry telemetry) {
        flywheel = new MotorEx(hm, "flywheel");

        this.telemetry = telemetry;

        flywheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        flywheel.encoder.reset();

        controller = new PIDFController(
                pidfValues.kP,
                pidfValues.kI,
                pidfValues.kD,
                pidfValues.kF
        );

        pidfValues.setLasts();
    }

    public void setPower(double power) {
        flywheel.set(Math.max(-1.0, Math.min(1.0, power)));
    }

    public double getPower () {
        return flywheel.get();
    }

    public double getTicksPerSec() { return flywheel.encoder.getRawVelocity(); }
    public double getRPM() { return (getTicksPerSec() * 60.0) / TICKS_PER_REV;}

    public void setTargetRPM (double rpm) {
        targetRPM = Math.max(0, rpm);
    }

    public double getTargetRPM () {
        return targetRPM;
    }

    /**
     * @param enablePID true if you want the PID on, false if off
     */
    public void setEnablePID (boolean enablePID) {
        FlywheelConfig.enablePID = enablePID;
    }

    public boolean getEnablePID () {
        return enablePID;
    }

    private void sendTelemetry () {
        telemetry.addData("Current RPM: ", getRPM());
        telemetry.addData("Is PID controller on: ", getEnablePID());
    }

    @Override
    public void periodic () {
        updatePIDController(controller, pidfValues);

        if (enablePID) {
            double pidOut = controller.calculate(getRPM(), targetRPM);
            setPower(pidOut);
        } else {
            setPower(targetPower);
        }

        sendTelemetry();
    }
}
