package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.config.HardwareNames;

public class IntakeSubsystem implements Subsystem {
    public static final double INTAKE_POWER=1;
    public static final double OUTAKE_POWER=-1;
    public static final double STOP_POWER=0;
    public static MotorEx intake;
    public IntakeSubsystem(HardwareMap hm){
        intake=new MotorEx(hm, "intake");


    }
    public void setPower (double power){
        intake.set(power);
    }
    public double getPower(){
        return intake.get();
    }
}
