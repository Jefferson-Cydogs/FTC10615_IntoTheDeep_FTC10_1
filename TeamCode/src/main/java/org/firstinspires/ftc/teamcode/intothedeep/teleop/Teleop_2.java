package org.firstinspires.ftc.teamcode.intothedeep.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Teleop_2 extends LinearOpMode {

    // Extension is the Intake box
    private Servo Extension;
    private Servo DeliveryBox;
//Intakebox is the continuous servo
private CRServo IntakeBox;
    private double servoPosition = 0.3;
private double servoPosition1 = 0.7;

    @Override
    public void runOpMode() {
        {
            Extension = hardwareMap.get(Servo.class, "Extension");

            DeliveryBox = hardwareMap.get(Servo.class, "DeliveryBox");

            IntakeBox = hardwareMap.get(CRServo.class, "IntakeBox");

            DeliveryBox.setPosition(servoPosition);
            Extension.setPosition(servoPosition1);



            waitForStart();

            while (opModeIsActive()) {
                // do op mode things here

                if (gamepad1.square) {
                    Extension.setPosition(0.1);
                    //   sleep(2000);
                } else if (gamepad1.triangle) {
                    Extension.setPosition(servoPosition1);


                }
                if (gamepad1.circle) {
                    DeliveryBox.setPosition(0.95);
                    //   sleep(2000);
                } else if (gamepad1.cross) {
                    DeliveryBox.setPosition(servoPosition);


                }
                if (gamepad1.dpad_up) {
                    IntakeBox.setPower(0.5);
                    //   sleep(2000);
                }
                else if(gamepad1.dpad_down) {
                    IntakeBox.setPower(-0.5);
                    // sleep(2000);
                }
                else {
                    IntakeBox.setPower(0);
                }



            }
        }

    }
}