package org.firstinspires.ftc.teamcode.commands.turretpos;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.TurretRotSubsystem;

public class SetPosTurretCommand extends CommandBase {

    private final TurretRotSubsystem turretRot;

    private final double angleDegrees;

    /** @param angleDegrees signed turret angle in degrees relative to the startup reference */
    public SetPosTurretCommand(TurretRotSubsystem turretRot, double angleDegrees){

        this.turretRot = turretRot;
        this.angleDegrees = angleDegrees;

        addRequirements(turretRot);
    }
    @Override
    public void initialize(){turretRot.turnToAngle(angleDegrees);}


    @Override
    public boolean isFinished(){

        return true;
    }
}