package org.firstinspires.ftc.teamcode.robotTests.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.TestUtils;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

public class IntakeSubsystemTest {
    public IntakeSubsystem createIntakeSubsystem() {
        TestUtils.MockHardwareMap hardwareMap = new TestUtils.HardwareMapBuilder()
                .withMotor("intake")
                .build();

        return TestUtils.createSubsystem(IntakeSubsystem::new, hardwareMap);
    }

    @Test
    @DisplayName("Test Subsystem creation and motor/servo state management")
    public void testIntakeSubsystemBehavior() {
        IntakeSubsystem subsystem = createIntakeSubsystem();
        assertNotNull(subsystem, "Subsystem should be instantiated successfully");

        IntakeSubsystem.DASHBOARD_CONTROL = false;
        // 1. Test setPower when DASHBOARD_CONTROL is false
        subsystem.setPower(0.85);
        subsystem.periodic(); // Applies setPower(0.85) to the intake motor
        assertEquals(0.85, subsystem.getPower(), 1e-6, "Motor power should match set value");

        // 2. Test DASHBOARD_CONTROL override
        IntakeSubsystem.DASHBOARD_CONTROL = true;
        IntakeSubsystem.DASHBOARD_POWER = 0.5;

        subsystem.setPower(0.0); // Change setPower to 0
        subsystem.periodic();    // Applies DASHBOARD_POWER to the intake motor
        assertEquals(0.5, subsystem.getPower(), 1e-6, "Motor power should use DASHBOARD_POWER when DASHBOARD_CONTROL is true");
    }

    @Test
    @DisplayName("Test Subsystem creation fails when required hardware is missing from HardwareMap")
    public void testMissingHardwareHandling() {
        TestUtils.MockHardwareMap emptyMap = new TestUtils.MockHardwareMap();
        assertThrows(
                IllegalArgumentException.class,
                () -> new IntakeSubsystem(emptyMap, TestUtils.getMultipleTelemetry()),
                "Subsystem creation should fail if required motor or servo names are missing from HardwareMap"
        );
    }
}