package org.firstinspires.ftc.teamcode.commands.flywheel;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.FlyWheelSubsystem;

public class SetPowerFlywheelCommand extends CommandBase {

    private final FlyWheelSubsystem flyWheel;
    private final double power;

    public SetPowerFlywheelCommand(FlyWheelSubsystem flyWheel,double power){
        this.flyWheel = flyWheel;
        this.power = power;

        addRequirements(flyWheel);
    }
    @Override
    public void initialize() {
        flyWheel.setPower(power);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
