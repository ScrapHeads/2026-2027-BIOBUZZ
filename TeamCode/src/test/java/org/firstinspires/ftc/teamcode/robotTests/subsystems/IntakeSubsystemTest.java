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
    public void IntakeSubsystemBehavior() {
        IntakeSubsystem subsystem = createIntakeSubsystem();
        assertNotNull(subsystem, "Subsystem should be instantiated successfully");

        // Verify motor power controls
        subsystem.setPower(0.85);
        assertEquals(0.85, subsystem.getPower(), 1e-6, "Motor power should match set value");
    }
    @Test
    @DisplayName("Test Subsystem creation fails when required hardware is missing from HardwareMap")
    public void testMissingHardwareHandling() {
        TestUtils.MockHardwareMap emptyMap = new TestUtils.MockHardwareMap();
        assertThrows(
                IllegalArgumentException.class,
                () -> new IntakeSubsystem(emptyMap),
                "Subsystem creation should fail if required motor or servo names are missing from HardwareMap"
        );
    }
}