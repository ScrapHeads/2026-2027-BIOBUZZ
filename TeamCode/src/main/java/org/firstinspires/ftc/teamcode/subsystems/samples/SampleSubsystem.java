package org.firstinspires.ftc.teamcode.subsystems.samples;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

/**
 * Sample Subsystem incorporating both a MotorEx ("sampleMotor") and a Servo ("sampleServo").
 * Serves as a reference implementation for creating robot subsystems and writing unit tests.
 */
public class SampleSubsystem implements Subsystem {

    private final MotorEx motor;
    private final ServoEx servo;

    public SampleSubsystem(HardwareMap hardwareMap) {
        this.motor = new MotorEx(hardwareMap, "sampleMotor");
        this.servo = new ServoEx(hardwareMap, "sampleServo");
    }

    public void setMotorPower(double power) {
        motor.set(power);
    }

    public double getMotorPower() {
        return motor.get();
    }

    public void setServoPosition(double position) {
        servo.set(position);
    }

    public double getServoPosition() {
        return servo.get();
    }
}
