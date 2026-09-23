package org.firstinspires.ftc.teamcode.commands.transfer;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Transfer;

public class SetPowerTransfer extends CommandBase {
    private final Transfer transfer;
    private final double power;
    public SetPowerTransfer (Transfer transfer, double power){
        this.transfer = transfer;
        this.power = power;

        addRequirements(transfer);
    }
    @Override
    public void initialize(){
        transfer.setPower(power);
    }
    @Override
    public void end(boolean interrupted) {

    }
    @Override
    public boolean isFinished(){
        return true;
    }
}