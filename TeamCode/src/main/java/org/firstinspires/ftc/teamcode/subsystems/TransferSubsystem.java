package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class TransferSubsystem extends SubsystemBase {
    private final MotorEx transfer;

    public static final double TRANSFER_IN_POWER = 1;
    public static final double TRANSFER_LAUNCH_POWER = .90;
    public static final double TRANSFER_OUT_POWER = -.5;
    public static final double TRANSFER_OUT_POWER_SLOW = -.5;


    public TransferSubsystem(HardwareMap hm) {
        transfer = new MotorEx (hm, "transfer");
    }

     public void setPower (double power) {
        transfer.set(power);
    }

    public double getPower (){
        return transfer.get();
    }
}
