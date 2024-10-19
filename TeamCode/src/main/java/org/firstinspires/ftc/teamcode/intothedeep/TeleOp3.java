package org.firstinspires.ftc.teamcode.intothedeep;


import android.icu.text.Transliterator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TeleOp3 extends LinearOpMode {

    // declare variables here
    public Servo SpecimenGripper;

    public double servoposition =0;
    @Override
    public void runOpMode() {

        //    Do initialization things here

        SpecimenGripper=hardwareMap.get(Servo.class,"SpecimenGripper");
        waitForStart();

        while (opModeIsActive()) {
            // do op mode things here

            if(gamepad2.right_bumper){
                servoposition =0.8;
                SpecimenGripper.setPosition(servoposition);
            }

            if(gamepad2.left_bumper){
               servoposition =0;
                SpecimenGripper.setPosition(servoposition);
           }
        }



    }
}
