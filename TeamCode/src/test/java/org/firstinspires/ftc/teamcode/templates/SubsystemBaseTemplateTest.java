package org.firstinspires.ftc.teamcode.templates;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.firstinspires.ftc.teamcode.subsystems.samples.SampleSubsystem;
import org.firstinspires.ftc.teamcode.util.TestUtils;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

/**
 * Base template test for Subsystems.
 * Demonstrates testing subsystem initialization, motor power, servo positioning,
 * and handling missing hardware using SampleSubsystem and TestUtils.
 */
public class SubsystemBaseTemplateTest {

    public SampleSubsystem createSampleSubsystem() {
        TestUtils.MockHardwareMap hardwareMap = new TestUtils.HardwareMapBuilder()
                .withMotor("sampleMotor")
                .withServo("sampleServo")
                .build();
        return TestUtils.createSubsystem(SampleSubsystem::new, hardwareMap);
    }

    @Test
    @DisplayName("Test Subsystem creation and motor/servo state management")
    public void testSubsystemBehavior() {
        SampleSubsystem subsystem = createSampleSubsystem();
        assertNotNull(subsystem, "Subsystem should be instantiated successfully");

        // Verify motor power controls
        subsystem.setMotorPower(0.85);
        assertEquals(0.85, subsystem.getMotorPower(), 1e-6, "Motor power should match set value");

        // Verify servo position controls
        subsystem.setServoPosition(0.4);
        assertEquals(0.4, subsystem.getServoPosition(), 1e-6, "Servo position should match set value");
    }

    @Test
    @DisplayName("Test Subsystem creation fails when required hardware is missing from HardwareMap")
    public void testMissingHardwareHandling() {
        TestUtils.MockHardwareMap emptyMap = new TestUtils.MockHardwareMap();
        assertThrows(
                IllegalArgumentException.class,
                () -> new SampleSubsystem(emptyMap),
                "Subsystem creation should fail if required motor or servo names are missing from HardwareMap"
        );
    }
}
