package org.firstinspires.ftc.teamcode.robotTests.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.commands.transfer.SetPowerTransferCommand;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.util.TestUtils;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

public class SetPowerTransferCommandTest {
    public TransferSubsystem createTransferSubsystem() {
        TestUtils.MockHardwareMap hardwareMap = new TestUtils.HardwareMapBuilder()
                .withMotor("transfer")
                .build();
        return TestUtils.createDummyTransferSubsystem();
    }

    @Test
    @DisplayName("Test SampleCommand initializes, executes subsystem actions, and finishes")
    public void testTransferSetPowerCommandExecution() {
        TransferSubsystem subsystem = createTransferSubsystem();

        double targetPower = 0.75;
        SetPowerTransferCommand command = new SetPowerTransferCommand(subsystem, targetPower);

        // Execute command initialization phase
        command.initialize();

        // Verify subsystem state changes after command runs
        assertEquals(targetPower, subsystem.getPower(), 1e-6, "Subsystem motor power should equal target value");
        assertTrue(command.isFinished(), "SampleCommand should be finished");
    }
}
