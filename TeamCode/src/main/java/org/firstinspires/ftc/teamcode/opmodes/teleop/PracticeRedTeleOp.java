package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "PracticeRedTeleOp", group = "ScrapHeads")
public class PracticeRedTeleOp extends BaseTeleOp {

    @Override
    protected Boolean getAllianceTeam() {
        return false; // Red Alliance
    }
}
