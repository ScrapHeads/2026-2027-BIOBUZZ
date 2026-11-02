package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class FlowerRampSubsystem extends SubsystemBase {

    private final ServoEx servo1;

    public static final double MIN_ANGLE_SERVO = 500;
    public static  final double MAX_ANGLE_SERVO = 2500;

    public static final double DOWN_POS = 1820;
    public static final double UP_POS = 2500;

    public enum RampState {
        UP,
        DOWN
    }

    private RampState currentState = RampState.UP;

    public FlowerRampSubsystem(HardwareMap hm) {
        this.servo1 = new ServoEx(hm, "ramp", MIN_ANGLE_SERVO, MAX_ANGLE_SERVO);
        setRampState(RampState.UP);
    }

    public void setRampState(RampState state) {
        this.currentState = state;
        switch (state) {
            case UP:
                servo1.set(UP_POS);
                break;
            case DOWN:
                servo1.set(DOWN_POS);
                break;
        }
    }

    public void toggleRampState() {
        if (currentState == RampState.UP) {
            setRampState(RampState.DOWN);
        } else {
            setRampState(RampState.UP);
        }
    }

    public RampState getRampState() {
        return currentState;
    }

    public void setFlowerRampPos(double position) {
        servo1.set(position);
    }

    public double getFlowerRampPos() {
        return servo1.get();
    }
}
