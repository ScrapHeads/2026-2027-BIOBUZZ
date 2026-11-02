package org.firstinspires.ftc.teamcode.commands.flowerRamp;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.FlowerRampSubsystem;

public class setFlowerRampPosCommand extends CommandBase {
    private final FlowerRampSubsystem ramp;
    private final double pos;

    public setFlowerRampPosCommand(FlowerRampSubsystem ramp, double pos) {
        this.ramp = ramp;
        this.pos = pos;
        addRequirements(ramp);
    }

    @Override
    public void initialize(){
        ramp.setFlowerRampPos(pos);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
