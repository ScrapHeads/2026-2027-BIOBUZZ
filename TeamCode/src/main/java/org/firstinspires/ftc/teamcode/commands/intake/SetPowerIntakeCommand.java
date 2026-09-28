package org.firstinspires.ftc.teamcode.commands.intake;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
//calian
public class SetPowerIntakeCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final double power;
    public SetPowerIntakeCommand(IntakeSubsystem intake, double power){
        this.intake =intake;
        this.power = power;

        addRequirements(intake);
    }
    @Override
    public void initialize(){intake.setPower(power);}

    @Override
    public void end(boolean interrupted){

    }
    @Override
    public boolean isFinished(){

        return true;
    }

}
