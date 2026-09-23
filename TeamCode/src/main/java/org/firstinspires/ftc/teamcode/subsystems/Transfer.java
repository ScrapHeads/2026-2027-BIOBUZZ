package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class Transfer implements Subsystem {
    public static final double INTAKE_POWER = 1;
    public static final double OUTTAKE_POWER = -1;
    private static MotorEx transfer;
    public Transfer (HardwareMap hm) {
        transfer = new MotorEx (hm, "transfer");
    }
     public void setPower (double power) {
        transfer.set(power);
    }
    public double getPower (){
        return transfer.get();
    }


}
