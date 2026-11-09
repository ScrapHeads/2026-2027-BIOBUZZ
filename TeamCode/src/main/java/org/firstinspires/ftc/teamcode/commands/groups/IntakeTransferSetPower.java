package org.firstinspires.ftc.teamcode.commands.groups;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.intake.SetPowerIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.transfer.SetPowerTransferCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class IntakeTransferSetPower extends SequentialCommandGroup{

    double intake=0;
    double transfer=0;

    public IntakeTransferSetPower(IntakeSubsystem intake, TransferSubsystem transfer){
        addCommands(
                new SetPowerIntakeCommand(intake, 0),
                new SetPowerTransferCommand(transfer, 0)
        );
    }
}
