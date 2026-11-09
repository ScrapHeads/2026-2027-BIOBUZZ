package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotState;

public abstract class BaseTeleOp extends CommandOpMode {

    public Robot robot;

    public HardwareMap hm;
    public Telemetry tele;
    public GamepadEx driver1;
    public GamepadEx driver2;
    public MultipleTelemetry mTelemetry;

    /**
     * Override in practice OpModes to enforce alliance color:
     * return true for Blue, false for Red, or null to keep loaded auto state.
     */
    protected Boolean getAllianceTeam() {
        return null;
    }

    @Override
    public void initialize() {
        this.tele = telemetry;
        this.hm = hardwareMap;

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        robot = new Robot(hardwareMap, telemetry);
        robot.state.load();

        Boolean team = getAllianceTeam();
        if (team != null) {
            RobotState.getInstance().setTeam(team);
        }

        mTelemetry = robot.getTelemetry();

        assignControls();
    }

    @Override
    public void run() {
        super.run(); // VERY IMPORTANT: runs the CommandScheduler on every loop cycle

        // Flushes telemetry updates to BOTH Driver Hub and FTC Dashboard
        mTelemetry.update();
    }

    public void assignControls() {
        if (RobotState.getInstance().isBlueTeam()) {
            TeleOpControls.assignControls(robot, driver1, driver2);
        } else {
            TeleOpControls.assignControls(robot, driver2, driver1);
        }

    }
}
