package org.firstinspires.ftc.teamcode.commands.Lift;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class setPowerLift2Command extends CommandBase {
    private final LiftSubsystem lift;
    private final double power;

    public setPowerLift2Command(LiftSubsystem lift, double power) {
        this.lift = lift;
        this.power = power;

        addRequirements(lift);
    }
    @Override
    public void initialize(){lift.setLiftServo2Power(power);}

    @Override
    public void  end(boolean interrupted){

    }
    @Override
    public boolean isFinished(){

        return true;
    }
}