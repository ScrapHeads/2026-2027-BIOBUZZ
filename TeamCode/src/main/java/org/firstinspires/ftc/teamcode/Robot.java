package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.FlyWheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.StateIO;

public class Robot {
    public final HardwareMap hm;
    public final Telemetry telemetry;
    public static final FtcDashboard dashboard = FtcDashboard.getInstance();
    public final MultipleTelemetry multipleTelemetry;
    public final StateIO state;

    public final TransferSubsystem transfer;
    public final IntakeSubsystem intake;
    public final VisionSubsystem vision;

    public Robot (HardwareMap hm, Telemetry telemetry) {
        this.hm = hm;
        this.telemetry = telemetry;

        multipleTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        intake = new IntakeSubsystem(hm, multipleTelemetry);
        intake.register();

        transfer = new TransferSubsystem(hm);
        transfer.register();

        vision = new VisionSubsystem(hm, multipleTelemetry, dashboard);
        vision.register();

        state = new StateIO(multipleTelemetry, dashboard);
    }

    public MultipleTelemetry getTelemetry () {
        return multipleTelemetry;
    }
}
