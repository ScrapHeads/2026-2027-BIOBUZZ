package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.FlywheelConfig.pidfValues;
import static org.firstinspires.ftc.teamcode.config.FlywheelConfig.setByDistance;
import static org.firstinspires.ftc.teamcode.util.PIDUtil.updatePIDController;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class LiftSubsystem implements Subsystem {
    private final ServoEx liftServo1;
    private final ServoEx liftServo2;
    private final MotorEx liftEncoder;
    private final PIDFController controller;
    private final MultipleTelemetry telemetry;

    double targetPos = 0;
    boolean enablePID = false;

    public LiftSubsystem(HardwareMap hm, PIDFController controller, MultipleTelemetry telemetry){
        this.liftServo1 = new ServoEx(hm,"liftServo1");
        this.liftServo2 = new ServoEx(hm, "liftServo2");
        this.liftEncoder = new MotorEx(hm, "encoder");
        this.controller = controller;
        this.telemetry = telemetry;
        controller = new PIDFController(
                pidfValues.kP,
                pidfValues.kI,
                pidfValues.kD,
                pidfValues.kF
        );
        pidfValues.setLasts();
    }
    public void setLiftServo1Power(double power){
        liftServo1.set(power);}
    public double getLiftServo1Power() {
        return liftServo1.get();
    }
    public void setLiftServo2Power(double power){
        liftServo2.set(power);
    }
    public double getLiftServo2Power() {
        return liftServo2.get();
    }
    public void setLiftBoth(double power1, double power2){
        liftServo1.set(power1);
        liftServo2.set(power2);
    }
    public double getCurrentPosition(){

        return liftEncoder.encoder.getPosition();
    }
    public void setTargetPos(double newTargetPos){
        targetPos = newTargetPos;
    }
    public double getTargetPos(){
        return targetPos;
    }
    public void setEnablePID(boolean isEnabled){
        enablePID = isEnabled;
    }
    public boolean getEnablePID(){
        return enablePID;
    }
    private void sendTelemetry () {
        telemetry.addData("Current place: ", getTargetPos());
        telemetry.addData("Is PID controller on: ", getEnablePID());
    }
    @Override
    public void periodic () {
        updatePIDController(controller, pidfValues);

        double getTargetPos = 0;
        if (enablePID) {
            if (setByDistance) {

            }
            double pidOut = controller.calculate(getCurrentPosition(), getTargetPos);
            pidOut = pidOut > 0 ? pidOut : 0;
            setLiftBoth(pidOut, pidOut);
        } else {
            getTargetPos = getTargetPos();
            setLiftBoth(getTargetPos, getTargetPos);
        }

        sendTelemetry();
    }
}
