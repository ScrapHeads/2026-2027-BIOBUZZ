package org.firstinspires.ftc.teamcode.robotTests.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.util.TestUtils;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

public class TransferSubsystemTest {
    public TransferSubsystem createTransferSubsystem() {
        TestUtils.MockHardwareMap hardwareMap = new TestUtils.HardwareMapBuilder()
                .withMotor("transfer")
                .build();
        return TestUtils.createSubsystem(TransferSubsystem::new, hardwareMap);
    }

    @Test
    @DisplayName("Test Subsystem creation and motor/servo state management")
    public void testSubsystemBehavior() {
        TransferSubsystem subsystem = createTransferSubsystem();
        assertNotNull(subsystem, "Subsystem should be instantiated successfully");

        // Verify motor power controls
        subsystem.setPower(1);
        assertEquals(1, subsystem.getPower(), 1, "Motor power should match set value");
    }


    @Test
    @DisplayName("Test Subsystem creation fails when required hardware is missing from HardwareMap")
    public void testMissingHardwareHandling() {
        TestUtils.MockHardwareMap emptyMap = new TestUtils.MockHardwareMap();
        assertThrows(
                IllegalArgumentException.class,
                () -> new TransferSubsystem(emptyMap),
                "Subsystem creation should fail if required motor or servo names are missing from HardwareMap"
        );
    }
}
