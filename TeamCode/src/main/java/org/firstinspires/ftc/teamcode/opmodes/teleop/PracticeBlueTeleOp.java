package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "PracticeBlueTeleOp", group = "ScrapHeads")
public class PracticeBlueTeleOp extends BaseTeleOp {

    @Override
    protected Boolean getAllianceTeam() {
        return true; // Blue Alliance
    }
}
