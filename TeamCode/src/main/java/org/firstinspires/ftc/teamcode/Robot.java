package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlowerRampSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlyWheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeBlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretStopperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.StateIO;

public class Robot implements AutoCloseable {
    public final HardwareMap hm;
    public final Telemetry telemetry;
    public static final FtcDashboard dashboard = FtcDashboard.getInstance();
    public final MultipleTelemetry multipleTelemetry;
    public final StateIO state;

    public final DrivetrainSubsystem drivetrain;

    public final FlyWheelSubsystem flyWheel;

    public final TransferSubsystem transfer;
    public final IntakeSubsystem intake;

    public final VisionSubsystem vision;
    public final FlowerRampSubsystem ramp;
    public final IntakeBlockerSubsystem intakeBlocker;
    public final TurretStopperSubsystem turretStopper;

    public Robot (HardwareMap hm, Telemetry telemetry) {
        this.hm = hm;
        this.telemetry = telemetry;

        multipleTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        drivetrain = new DrivetrainSubsystem(hm, multipleTelemetry);

        flyWheel = new FlyWheelSubsystem(hm, multipleTelemetry);

        intake = new IntakeSubsystem(hm, multipleTelemetry);

        transfer = new TransferSubsystem(hm);

        vision = new VisionSubsystem(hm, multipleTelemetry, dashboard);
        ((org.firstinspires.ftc.teamcode.localization.DrivetrainLocalizer)
                drivetrain.follower.localizer).setVisionUpdate(vision::updateLocalization);

        turretStopper = new TurretStopperSubsystem(hm);

        ramp = new FlowerRampSubsystem(hm);

        intakeBlocker = new IntakeBlockerSubsystem(hm);

        state = new StateIO(multipleTelemetry, dashboard);
    }

    @Override public void close() {
        vision.close();
        drivetrain.stop();
    }

    public MultipleTelemetry getTelemetry () {
        return multipleTelemetry;
    }
}
