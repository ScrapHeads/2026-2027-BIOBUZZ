package org.firstinspires.ftc.teamcode.opmodes.teleop;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys.Trigger.*;
import static com.seattlesolvers.solverslib.gamepad.GamepadKeys.Button.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.intake.SetPowerIntake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commands.transfer.SetPowerTransfer;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;



@TeleOp(name = "MainTeleOp", group = "ScrapHeads")
public class MainTeleOp extends CommandOpMode {

    public Robot robot;

    public HardwareMap hm;
    public Telemetry tele;

    public GamepadEx driver1;
    public GamepadEx driver2;

    @Override
    public void initialize() {
        this.tele = telemetry;
        this.hm = hardwareMap;

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        robot = new Robot(hardwareMap, telemetry);

        assignControls();
    }
    public void  assignControls() {
        driver1.getGamepadButton(DPAD_RIGHT)
                .whenPressed(new SetPowerIntake(robot.intake, IntakeSubsystem.INTAKE_POWER));
        driver1.getGamepadButton(DPAD_LEFT)
                .whenPressed(new SetPowerIntake(robot.intake, IntakeSubsystem.OUTAKE_POWER));
        
        driver1.getGamepadButton(DPAD_UP)
                .whenPressed(new SetPowerTransfer(robot.transfer, Transfer.INTAKE_POWER));
        driver1.getGamepadButton(DPAD_DOWN)
                .whenPressed(new SetPowerTransfer(robot.transfer, Transfer.OUTTAKE_POWER));

        driver1.getGamepadButton(A)
                .whenPressed(new ParallelCommandGroup(
                        new SetPowerTransfer(robot.transfer, 0),
                        new SetPowerIntake(robot.intake, 0)
                ));
    }
}
