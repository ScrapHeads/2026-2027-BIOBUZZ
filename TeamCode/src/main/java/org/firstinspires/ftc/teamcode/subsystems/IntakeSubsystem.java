package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.config.HardwareNames;

@Config
public class IntakeSubsystem implements Subsystem {
    public static final double INTAKE_POWER=.75;
    public static final double OUTAKE_POWER=-.75;
    public static final double STOP_POWER=0;

    public final MultipleTelemetry telemetry;

    public static double power = 0.0;
    public static boolean DASHBOARD_CONTROL = true;
    public static double DASHBOARD_POWER = 0.0;

    /** Instance of params for this launcher. */
    public final MotorEx intake;
    public IntakeSubsystem(HardwareMap hm, MultipleTelemetry telemetry){
        intake = new MotorEx(hm, "intake");
        this.telemetry = telemetry;
    }
    public void setPower (double newPower){
        power = newPower;
    }
    public double getPower(){
        return intake.get();
    }

    @Override
    public void periodic () {

        double output = power;

        intake.set(output);

        telemetry.addData("Intake Target Power", power);
        telemetry.addData("Intake Actual Power", intake.get());
        telemetry.addData("Dashboard Power", DASHBOARD_CONTROL);
        telemetry.addData("Dashboard Power", DASHBOARD_POWER);
    }
}
