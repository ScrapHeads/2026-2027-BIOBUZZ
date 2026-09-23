package org.firstinspires.ftc.teamcode.opmodes.teleop;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys.Trigger.*;
import static com.seattlesolvers.solverslib.gamepad.GamepadKeys.Button.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.intake.SetPowerIntake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;


@TeleOp(name = "MainTeleOp", group = "ScrapHeads")
public class MainTeleOp extends CommandOpMode {

    public Robot robot;

    public HardwareMap hm;
    public Telemetry tele;

    @Override
    public void initialize() {
        this.tele = telemetry;
        this.hm = hardwareMap;

        robot = new Robot(hm, tele);

        assignControls();
    }
    public void  assignControls(){
        robot.driver1.getGamepadButton(DPAD_RIGHT)
                .whenPressed(new SetPowerIntake(robot.intake, IntakeSubsystem.INTAKE_POWER))
                .whenReleased(new SetPowerIntake(robot.intake, IntakeSubsystem.STOP_POWER));
        
    }
}
