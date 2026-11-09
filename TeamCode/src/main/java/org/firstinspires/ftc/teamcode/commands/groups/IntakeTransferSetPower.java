package org.firstinspires.ftc.teamcode.commands.groups;

import android.health.connect.datatypes.units.Power;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.intake.SetPowerIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.transfer.SetPowerTransferCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class IntakeTransferSetPower extends SequentialCommandGroup{

    public IntakeTransferSetPower(IntakeSubsystem intake, TransferSubsystem transfer, double intakePower, double transferPower){
        addCommands(
                new SetPowerIntakeCommand(intake, transferPower),
                new SetPowerTransferCommand(transfer, intakePower)
        );
    }
}
