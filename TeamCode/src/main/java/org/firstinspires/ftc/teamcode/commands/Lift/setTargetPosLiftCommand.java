package org.firstinspires.ftc.teamcode.commands.Lift;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class setTargetPosLiftCommand extends CommandBase {
    private final LiftSubsystem lift;


    public setTargetPosLiftCommand(LiftSubsystem lift) {
        this.lift = lift;
    }
}