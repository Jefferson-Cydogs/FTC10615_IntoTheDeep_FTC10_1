package org.firstinspires.ftc.teamcode.learning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class DeliveryBox extends LinearOpMode {

    // declare variables here
    private Servo DeliveryBox;

    private double servoPosition = 0.3;

    @Override
    public void runOpMode() {
        {
            DeliveryBox = hardwareMap.get(Servo.class, "DeliveryBox");

            DeliveryBox.setPosition(servoPosition);

            waitForStart();


            while (opModeIsActive()) {
                // do op mode things here

                if (gamepad1.a) {
                    DeliveryBox.setPosition(0.95);
                    //   sleep(2000);
                } else if (gamepad1.b) {
                    DeliveryBox.setPosition(servoPosition);


                }
            }
        }

    }
}