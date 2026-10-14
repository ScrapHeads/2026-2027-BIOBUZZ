package org.firstinspires.ftc.teamcode.commands.Lift;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class setPowerLiftCommand extends CommandBase {
    private final LiftSubsystem lift;
    private final double power;

    public setPowerLiftCommand(LiftSubsystem lift,double power) {
        this.lift = lift;
        this.power =power;

        addRequirements(lift);
    }
}
