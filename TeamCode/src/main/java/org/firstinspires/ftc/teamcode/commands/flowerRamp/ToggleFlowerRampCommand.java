package org.firstinspires.ftc.teamcode.commands.flowerRamp;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.FlowerRampSubsystem;

public class ToggleFlowerRampCommand extends CommandBase {
    private final FlowerRampSubsystem ramp;

    public ToggleFlowerRampCommand(FlowerRampSubsystem ramp) {
        this.ramp = ramp;
        addRequirements(ramp);
    }

    @Override
    public void initialize() {
        ramp.toggleRampState();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
