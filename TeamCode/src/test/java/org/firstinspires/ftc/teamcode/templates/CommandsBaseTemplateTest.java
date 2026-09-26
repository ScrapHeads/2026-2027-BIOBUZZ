package org.firstinspires.ftc.teamcode.templates;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.commands.samples.SampleCommand;
import org.firstinspires.ftc.teamcode.subsystems.samples.SampleSubsystem;
import org.firstinspires.ftc.teamcode.util.TestUtils;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

/**
 * Base template test for Commands.
 * Demonstrates testing command execution, subsystem state changes, and completion using SampleCommand.
 */
public class CommandsBaseTemplateTest {

    public SampleSubsystem createSampleSubsystem() {
        TestUtils.MockHardwareMap hardwareMap = new TestUtils.HardwareMapBuilder()
                .withMotor("sampleMotor")
                .withServo("sampleServo")
                .build();
        return TestUtils.createSubsystem(SampleSubsystem::new, hardwareMap);
    }

    @Test
    @DisplayName("Test SampleCommand initializes, executes subsystem actions, and finishes")
    public void testSampleCommandExecution() {
        SampleSubsystem subsystem = createSampleSubsystem();

        double targetPower = 0.75;
        double targetPosition = 0.6;
        SampleCommand command = new SampleCommand(subsystem, targetPower, targetPosition);

        // Execute command initialization phase
        command.initialize();

        // Verify subsystem state changes after command runs
        assertEquals(targetPower, subsystem.getMotorPower(), 1e-6, "Subsystem motor power should equal target value");
        assertEquals(targetPosition, subsystem.getServoPosition(), 1e-6, "Subsystem servo position should equal target value");
        assertTrue(command.isFinished(), "SampleCommand should be finished");
    }
}
