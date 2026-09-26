package org.firstinspires.ftc.teamcode.commands.samples;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.samples.SampleSubsystem;

/**
 * Sample Command operating on SampleSubsystem.
 * Demonstrates subsystem requirements, command initialization, and execution state.
 */
public class SampleCommand extends CommandBase {

    private final SampleSubsystem subsystem;
    private final double motorPower;
    private final double servoPosition;

    public SampleCommand(SampleSubsystem subsystem, double motorPower, double servoPosition) {
        this.subsystem = subsystem;
        this.motorPower = motorPower;
        this.servoPosition = servoPosition;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.setMotorPower(motorPower);
        subsystem.setServoPosition(servoPosition);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
