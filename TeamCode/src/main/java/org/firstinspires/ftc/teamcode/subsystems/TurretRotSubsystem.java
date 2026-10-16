package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.FlywheelConfig.enablePID;
import static org.firstinspires.ftc.teamcode.config.FlywheelConfig.pidfValues;
import static org.firstinspires.ftc.teamcode.config.FlywheelConfig.targetPower;
import static org.firstinspires.ftc.teamcode.config.pedro.TurretPosConfig.targetPos;
import static org.firstinspires.ftc.teamcode.util.PIDUtil.updatePIDController;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.config.pedro.TurretPosConfig;

public class TurretRotSubsystem implements Subsystem {
    private final MotorEx turretRot;

    private final PIDFController controller;

    private final MultipleTelemetry telemetry;

    public static final double MOTOR_TPR=1;//TODO
    public static final double GEAR_RATIO=1.0/3.0;//TODO

    public static final double TICKS_PER_REV = MOTOR_TPR / GEAR_RATIO;

    public TurretRotSubsystem(HardwareMap hm, MultipleTelemetry telemetry){
        turretRot = new MotorEx(hm,"turretRot");

        this.telemetry = telemetry;

        turretRot.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        turretRot.encoder.reset();

        controller= new PIDFController(
                pidfValues.kP,
                pidfValues.kI,
                pidfValues.kD,
                pidfValues.kF
        );
        pidfValues.setLasts();
    }
    public void setPower(double power){turretRot.set(Math.max(-1.0, Math.min(1.0, power)));}

    public double getPower (){return turretRot.get();}

    public double getTicksPerSec() { return turretRot.encoder.getRawVelocity();}

    public double getPos() { return(getTicksPerSec()* 60.0)/TICKS_PER_REV;}

    public void setTargetPos (double pos) {targetPos = Math.max(0, pos);}

    public double getTargetPos(){ return targetPos;}

    /**
     * @param enablePID true if you want the PID on, false if off
     */

    public void setEnablePID (boolean enablePID){TurretPosConfig.enablePID=enablePID;}

    public boolean getEnablePID (){return enablePID;}

    private void sendTelemetry () {
        telemetry.addData("Current Pos: ", getPos());
        telemetry.addData("Is PID controller on: ", getEnablePID());
    }
    @Override
    public void periodic () {
        updatePIDController(controller, pidfValues);

        if (enablePID) {
            double pidOut = controller.calculate(getPos(), targetPos);
            setPower(pidOut);
        } else {
            setPower(targetPower);
        }

        sendTelemetry();
    }
}
