package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class IntakeBlockerSubsystem extends SubsystemBase {
    private final ServoEx leftServo;
    private final ServoEx rightServo;

    public static final double RIGHT_LOCK = 0;
    public static final double RIGHT_OPEN = 1;
    public static final double LEFT_LOCK = 1;
    public static final double LEFT_OPEN = 0;

    public IntakeBlockerSubsystem(HardwareMap hardwareMap) {
        this.leftServo = new ServoEx(hardwareMap, "IBL", 0, 1);
        this.rightServo = new ServoEx(hardwareMap, "IBR", 0, 1);
        leftServo.set(LEFT_LOCK);
        rightServo.set(RIGHT_LOCK);
    }
    public void setIntakeBlockerServoPositionBoth(double leftPos, double rightPos){
        leftServo.set(leftPos);
        rightServo.set(rightPos);
    }

    public double getIntakeBlockerServoPosition2 (double position) {
        return rightServo.get();
    }
    public double getIntakeBlockerServoPosition1 (double position) {
        return leftServo.get();
    }
}
