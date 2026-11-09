package org.firstinspires.ftc.teamcode.commands.intakeBlocker;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeBlockerSubsystem;



public class ToggelBlockerCommand extends CommandBase {

    private final IntakeBlockerSubsystem blocker;
    public ToggelBlockerCommand(IntakeBlockerSubsystem blocker){
        this.blocker=blocker;
        addRequirements(blocker);
    }
    @Override
    public void initialize(){ blocker.toggleRampState();}

    @Override
    public boolean isFinished(){return true;}
}
