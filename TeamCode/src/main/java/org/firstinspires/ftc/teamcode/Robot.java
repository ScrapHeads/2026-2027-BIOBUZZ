package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.FlyWheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.util.StateIO;

public class Robot {
    public final HardwareMap hm;
    public final Telemetry telemetry;
    public final FtcDashboard dashboard;
    public final StateIO state;

    public final TransferSubsystem transfer;
    public final IntakeSubsystem intake;
    
    public Robot (HardwareMap hm, Telemetry telemetry) {
        this.hm = hm;
        this.telemetry = telemetry;
        this.dashboard = FtcDashboard.getInstance();

        intake = new IntakeSubsystem(hm);
        intake.register();

        transfer = new TransferSubsystem(hm);
        transfer.register();

        state = new StateIO(telemetry, dashboard);
    }

    public MultipleTelemetry getTelemetry () {
        return new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }
}
