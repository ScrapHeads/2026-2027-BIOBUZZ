package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@Config
public class IntakeSubsystem implements Subsystem {

    private final MotorEx intake;
    private final MultipleTelemetry telemetry;

    public static double power = 0.0;
    public volatile static boolean DASHBOARD_CONTROL = false;
    public volatile static double DASHBOARD_POWER = 0.0;

    public static final double INTAKE_POWER = 1;
    public static final double OUTTAKE_POWER = -1;
    public static final double STOP_POWER = 0;

    /** Instance of params for this launcher. */
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
        if (DASHBOARD_CONTROL) {
            output = DASHBOARD_POWER;
        }
        intake.set(output);

        sendTelemetry();
    }

    private void sendTelemetry () {
        telemetry.addData("Intake Target Power", power);
        telemetry.addData("Intake Actual Power", intake.get());
        telemetry.addData("Dashboard Power On", DASHBOARD_CONTROL);
        telemetry.addData("Dashboard Power", DASHBOARD_POWER);

        telemetry.addData("Dashboard Power On class", IntakeSubsystem.DASHBOARD_CONTROL);
        telemetry.addData("Dashboard Power class", IntakeSubsystem.DASHBOARD_POWER);
    }
}
