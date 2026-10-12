package org.firstinspires.ftc.teamcode.commands.flywheel;

import com.acmerobotics.dashboard.canvas.Rotation;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.FlyWheelSubsystem;

public class SetRPMFlywheelCommand extends CommandBase {

    private final FlyWheelSubsystem flyWheel;

    private final double rpm;

    public SetRPMFlywheelCommand(FlyWheelSubsystem flyWheel,double rpm){

        this.flyWheel=flyWheel;

        this.rpm=rpm;


        addRequirements(flyWheel);
    }
    @Override
    public void initialize(){flyWheel.setTargetRPM(rpm);}


    @Override
    public boolean isFinished(){

        return true;
    }
}