package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class LiftSubsystem implements Subsystem {
    private final ServoEx liftServo1;
    private final ServoEx liftServo2;
    private final MotorEx liftEncoder;

    double targetPos = 0;
    boolean enablePID = false;

    public LiftSubsystem(HardwareMap hm){
        this.liftServo1 = new ServoEx(hm,"liftServo1");
        this.liftServo2 = new ServoEx(hm, "liftServo2");
        this.liftEncoder = new MotorEx(hm, "encoder");
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
    public void getCurrentPosition(){
        liftEncoder.encoder.getPosition();
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
}
