package org.firstinspires.ftc.teamcode.commands.turretpos;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.TurretRotSubsystem;

public class SetPosTurretCommand extends CommandBase {

    private final TurretRotSubsystem turretRot;

    private final double pos;

    public SetPosTurretCommand(TurretRotSubsystem turretRot, double pos){

        this.turretRot = turretRot;
        this.pos=pos;

        addRequirements(turretRot);
    }
    @Override
    public void initialize(){turretRot.setTargetPos(pos);}


    @Override
    public boolean isFinished(){

        return true;
    }
}