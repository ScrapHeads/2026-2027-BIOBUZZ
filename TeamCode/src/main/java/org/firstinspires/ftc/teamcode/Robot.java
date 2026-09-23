package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.util.StateIO;

public class Robot {
    public final HardwareMap hm;
    public final Telemetry telemetry;
    public final FtcDashboard dashboard;

    public final GamepadEx driver1;
    public final GamepadEx driver2;

    public final Transfer transfer;
    public final IntakeSubsystem intake;

    public final StateIO state;
    public Robot (HardwareMap hm, Telemetry telemetry) {
        this.hm = hm;
        this.telemetry = telemetry;
        dashboard = FtcDashboard.getInstance();

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        intake = new IntakeSubsystem(hm);
        intake.register();

        transfer = new Transfer(hm);
        transfer.register();

        state = new StateIO(telemetry, dashboard);
    }

}
