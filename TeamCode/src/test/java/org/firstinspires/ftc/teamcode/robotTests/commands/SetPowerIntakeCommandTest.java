package org.firstinspires.ftc.teamcode.robotTests.commands;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.commands.intake.SetPowerIntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.TestUtils;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
/**
 * Base template test for Commands.
 * Demonstrates testing command execution, subsystem state changes, and completion using SampleCommand.
 */
public class SetPowerIntakeCommandTest {
    public IntakeSubsystem createIntakeSubsystem() {
        TestUtils.MockHardwareMap hardwareMap = new TestUtils.HardwareMapBuilder()
                .withMotor("intake")
                .build();
        return TestUtils.createSubsystem(IntakeSubsystem::new, hardwareMap);
    }
    @Test
    @DisplayName("Test SampleCommand initializes, executes subsystem actions, and finishes")
    public void testSampleCommandExecution() {
        IntakeSubsystem subsystem = createIntakeSubsystem();
        double targetPower = 0.75;
        SetPowerIntakeCommand command = new SetPowerIntakeCommand(subsystem, targetPower);
        command.initialize();
        assertEquals(targetPower, subsystem.getPower(), 1e-6, "Subsystem motor power should equal target value");
        assertTrue(command.isFinished(), "SampleCommand should be finished");
    }
}
