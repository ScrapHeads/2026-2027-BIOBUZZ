package org.firstinspires.ftc.teamcode.commands.Lift;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class setTargetPosLiftCommand extends CommandBase {
    private final LiftSubsystem lift;

    private final double targetPos;
    private final double liftPos;


    public setTargetPosLiftCommand(LiftSubsystem lift, double targetPos, double liftPos) {
        this.lift = lift;
        this.targetPos = targetPos;
        this.liftPos = liftPos;
        addRequirements(lift);
    }
    @Override
    public void initialize(){
        lift.setTargetPos(targetPos);
        lift.setLiftBoth(liftPos,liftPos);
    }
    @Override
    public boolean isFinished(){return true;}
}