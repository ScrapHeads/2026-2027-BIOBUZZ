package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class LiftSubsystem implements Subsystem {
    public final ServoEx liftServo1;
    public final ServoEx liftServo2;
    public final MotorEx liftEncoder;
    public LiftSubsystem(HardwareMap hm){
        this.liftServo1 = new ServoEx(hm,"liftServo1");
        this.liftServo2 = new ServoEx(hm, "liftServo2");
        this.liftEncoder =new MotorEx(hm, "encoder");
    }
    public void setLiftServo1Power(double Power){
        liftServo1.set(Power);}
    public double getLiftServo1Power(){return liftServo1.get();}
    public void setLiftServo2Power(double Power){
        liftServo2.set(Power);}
    public double getLiftServo2Power(){return liftServo2.get();}
    public void setLiftBoth(double Power){
        liftServo1.set(Power);
        liftServo2.set(Power);}
    public void getEncoderPosition(){
        liftEncoder.encoder.getPosition();
    }
}
