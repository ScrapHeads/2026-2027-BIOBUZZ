package org.firstinspires.ftc.teamcode.subsystems;

import android.icu.text.Transliterator;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.SimpleServo;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class TurretStopperSubsystem extends SubsystemBase {
    private final ServoEx stopper;

    public static final double MIN_ANGLE_SERVO = 1;
    public static  final double MAX_ANGLE_SERVO = 255;

    public static final double CLOSED_POS = 200;//todo

    public static final double OPEN_POS = 100;//todo

    public TurretStopperSubsystem (HardwareMap hm){
        stopper = new ServoEx(hm, "stopper", MIN_ANGLE_SERVO, MAX_ANGLE_SERVO);
        stopper.set(CLOSED_POS);

    }

    public void setPos (double newPos){stopper.set(newPos);}

    public double getPos(){return stopper.get();}
}