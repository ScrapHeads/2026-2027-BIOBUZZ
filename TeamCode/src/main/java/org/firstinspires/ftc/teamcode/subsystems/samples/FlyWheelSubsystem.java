package org.firstinspires.ftc.teamcode.subsystems.samples;

import static org.firstinspires.ftc.teamcode.config.FlywheelConfig.TICKS_PER_REV;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class FlyWheelSubsystem implements Subsystem {

    private static MotorEx flywheel;

    public FlyWheelSubsystem(HardwareMap hm) {
        flywheel = new MotorEx(hm, "flywheel");
    }
    public void setPower(double power) {
        flywheel.set(power);
    }
    public double getPower () {
        return flywheel.get();
    }

    public double getTicksPerSec() { return flywheel.encoder.getRawVelocity(); }
    public double getShooterRPM() { return (getTicksPerSec() * 60) / TICKS_PER_REV;}

    public double targetRPM () {
    return 0.0;
    }
}
