package org.firstinspires.ftc.teamcode.commands.Lift;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class setPowerLIft1Command extends CommandBase {
    private final LiftSubsystem lift;
    private final double power;

    public setPowerLIft1Command(LiftSubsystem lift, double power) {
        this.lift = lift;
        this.power = power;

        addRequirements(lift);
    }
    @Override
    public void initialize(){lift.setLiftServo1Power(power);}

    @Override
    public void  end(boolean interrupted){

    }
    @Override
    public boolean isFinished(){

        return true;
    }
}
