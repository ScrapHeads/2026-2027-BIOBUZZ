package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class IntakeBlockerSubsystem implements Subsystem{


    private final ServoEx servo1;
    private final ServoEx servo2;

    public IntakeBlockerSubsystem(HardwareMap hardwareMap) {
        this.servo1 = new ServoEx(hardwareMap, "IntakeBlockerServo2");
        this.servo2 = new ServoEx(hardwareMap, "IntakeBlockerServo1");
    }
    public void setIntakeBlockerServoPositionBoth(double position){servo1.set(position);servo2.set(position);}

    public double getIntakeBlockerServoPosition2(double position){return servo2.get();}
    public double getIntakeBlockerServoPosition1(double position){return servo1.get();}
}
