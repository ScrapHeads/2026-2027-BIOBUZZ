package org.firstinspires.ftc.teamcode.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.samples.SampleSubsystem;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

public class TestUtilsTest {

    @Test
    @DisplayName("Test TestUtils createMockMotor")
    public void testCreateMockMotor() {
        DcMotorEx motor = TestUtils.createMockMotor();
        assertNotNull(motor);
        motor.setPower(0.5);
        assertEquals(0.5, motor.getPower(), 1e-6);
    }

    @Test
    @DisplayName("Test TestUtils createMockServo")
    public void testCreateMockServo() {
        Servo servo = TestUtils.createMockServo();
        assertNotNull(servo);
        servo.setPosition(0.8);
        assertEquals(0.8, servo.getPosition(), 1e-6);
        servo.setDirection(Servo.Direction.REVERSE);
        assertEquals(Servo.Direction.REVERSE, servo.getDirection());
    }

    @Test
    @DisplayName("Test TestUtils createMockTelemetry")
    public void testCreateMockTelemetry() {
        Telemetry telemetry = TestUtils.createMockTelemetry();
        assertNotNull(telemetry);
        telemetry.addData("status", "running");
        telemetry.update();
    }

    @Test
    @DisplayName("Test HardwareMapBuilder with custom device names")
    public void testHardwareMapBuilder() {
        TestUtils.MockHardwareMap hm = new TestUtils.HardwareMapBuilder()
                .withMotor("frontLeft")
                .withMotor("frontRight")
                .withServo("claw")
                .withDevice("customMotor", TestUtils.createMockMotor())
                .build();

        assertNotNull(hm.get(DcMotorEx.class, "frontLeft"));
        assertNotNull(hm.get(DcMotorEx.class, "frontRight"));
        assertNotNull(hm.get(Servo.class, "claw"));
        assertNotNull(hm.get(DcMotorEx.class, "customMotor"));
    }

    @Test
    @DisplayName("Test createHardwareMapWithMotors and createHardwareMapWithServos")
    public void testHardwareMapConvenienceMethods() {
        TestUtils.MockHardwareMap motorHm = TestUtils.createHardwareMapWithMotors("m1", "m2");
        assertNotNull(motorHm.get(DcMotorEx.class, "m1"));
        assertNotNull(motorHm.get(DcMotorEx.class, "m2"));

        TestUtils.MockHardwareMap servoHm = TestUtils.createHardwareMapWithServos("s1", "s2");
        assertNotNull(servoHm.get(Servo.class, "s1"));
        assertNotNull(servoHm.get(Servo.class, "s2"));
    }

    @Test
    @DisplayName("Test generic createSubsystem method with constructor references")
    public void testGenericCreateSubsystem() {
        IntakeSubsystem intake = TestUtils.createSubsystem(IntakeSubsystem::new, "intake");
        assertNotNull(intake);

        Transfer transfer = TestUtils.createSubsystem(Transfer::new, "transfer");
        assertNotNull(transfer);

        TestUtils.MockHardwareMap sampleMap = new TestUtils.HardwareMapBuilder()
                .withMotor("sampleMotor")
                .withServo("sampleServo")
                .build();
        SampleSubsystem sampleSubsystem = TestUtils.createSubsystem(SampleSubsystem::new, sampleMap);
        assertNotNull(sampleSubsystem);
    }
}
