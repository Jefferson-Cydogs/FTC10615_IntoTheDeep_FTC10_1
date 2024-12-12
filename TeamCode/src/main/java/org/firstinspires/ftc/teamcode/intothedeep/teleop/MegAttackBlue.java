package org.firstinspires.ftc.teamcode.intothedeep.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="MEG Attack! Blue", group="Teleop")
public class MegAttackBlue extends MegAttackTeleop2{
    @Override
    public void runOpMode() {
        this.AllianceColor = "Blue";
        super.runOpMode();

    }
}
