package org.firstinspires.ftc.teamcode.learning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class IntakeBox extends LinearOpMode {

    // declare variables here
    private Servo Extension;

    private double servoPosition = 0.3;

    @Override
    public void runOpMode() {
        {
            Extension = hardwareMap.get(Servo.class, "Extension");
            Extension.setDirection(Servo.Direction.REVERSE);
            Extension.setPosition(servoPosition);



            waitForStart();

            while (opModeIsActive()) {
                // do op mode things here

                if (gamepad1.a) {
                    Extension.setPosition(0.7);
                    //   sleep(2000);
                } else if (gamepad1.b) {
                    Extension.setPosition(servoPosition);


                }
            }
        }

    }
}