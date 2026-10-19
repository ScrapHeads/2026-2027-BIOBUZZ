package org.firstinspires.ftc.teamcode.commands.turretstopper;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.teamcode.subsystems.TurretStopperSubsystem;

public class TurretStopperCommand extends CommandBase {

    private final TurretStopperSubsystem turretStopper;

    private final double pos;

    public TurretStopperCommand(TurretStopperSubsystem turretStopper, double pos){

        this.turretStopper = turretStopper;
        this.pos = pos;

        addRequirements((Subsystem) turretStopper);
    }
    @Override
    public void initialize(){turretStopper.setPos(pos);}


    @Override
    public boolean isFinished(){

        return true;
    }
}