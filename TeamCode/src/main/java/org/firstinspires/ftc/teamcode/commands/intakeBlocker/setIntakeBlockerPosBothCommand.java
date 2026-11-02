package org.firstinspires.ftc.teamcode.commands.intakeBlocker;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeBlockerSubsystem;

public class setIntakeBlockerPosBothCommand extends CommandBase {
    private final IntakeBlockerSubsystem blocker;
    private final double leftPos;
    private final double rightPos;

    public setIntakeBlockerPosBothCommand(IntakeBlockerSubsystem blocker, double leftPos, double rightPos) {
        this.blocker = blocker;
        this.leftPos = leftPos;
        this.rightPos = rightPos;
        addRequirements(blocker);
    }

    @Override
    public void initialize(){
        blocker.setIntakeBlockerServoPositionBoth(leftPos, rightPos);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
