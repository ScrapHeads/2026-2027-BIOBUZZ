package org.firstinspires.ftc.teamcode.commands.intakeBlocker;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeBlockerSubsystem;

public class setIntakeBlockerBothPositionCommand extends CommandBase {


    private final IntakeBlockerSubsystem subsystem;
    private final double servoPosition;

    public setIntakeBlockerBothPositionCommand(IntakeBlockerSubsystem subsystem, double servoPosition) {
        this.subsystem = subsystem;
        this.servoPosition = servoPosition;
        addRequirements(subsystem);
    }
    @Override
    public void initialize(){
        subsystem.setIntakeBlockerServoPositionBoth(servoPosition);
    }
    @Override
    public boolean isFinished(){return true;}
}
