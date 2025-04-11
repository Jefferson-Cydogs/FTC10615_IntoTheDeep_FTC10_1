package org.firstinspires.ftc.teamcode.configs;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



/*
This allows you to select which servos to test during initialization.
Using PlayStation controller buttons, Includes error handling to make sure servos are configured properly.
After selection you can control servos position using gamepad.
*/


@TeleOp
public class Config_Servo2 extends LinearOpMode {


    //Declare the servos

    //Varaible to keep track of the servo
    private Servo selectedServo = null;
    private String selectedServoName = "None";

    private boolean updated = false;

    //Define the initial servoPosition
    private double servoPosition = 0.5;

    //Timer for optional servo select time out
    private ElapsedTime selectionTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        waitForStart();

        //Display selection Instructions during Intialzation
        telemetry.addLine("Select Servo to Test using PlayStation Controller Buttons.")
                .addData("\nCross(X)", "Extension")
                .addData("\nCircle(O)", "Delivery Box")
                .addData("\nSquare(N)", "Specimen Gripper")
                .addData("\nTriangle(A)", "Specimen Gripper Rotator")
                .addData("\nCurrent Selection", selectedServoName);
        telemetry.update();





//Allows user to select a servo before TeleOp
        while (opModeIsActive() && selectedServo == null) {
//Check for servo selection buttons based on PlayStation controller mapping
            if (gamepad1.a) {//Cross(X)button
                selectedServoName = "Extension";
                selectedServo = initalizeServo(selectedServoName);
                telemetry.addData("Selected Servo", selectedServoName);
                telemetry.update();
                sleep(300);//delay to prevent multiple selections
            } else if (gamepad1.b) {//Circle button
                selectedServoName = "DeliveryBox";
                selectedServo = initalizeServo(selectedServoName);
                telemetry.addData("Selected Servo", selectedServoName);
                telemetry.update();
                sleep(300);
            } else if (gamepad1.x) {//Square button
                selectedServoName = "SpecimenGripper";
                selectedServo = initalizeServo(selectedServoName);
                selectedServo.setDirection(Servo.Direction.REVERSE);
                telemetry.addData("Selected Servo", selectedServoName);
                telemetry.update();
                sleep(300);
            } else if (gamepad1.y) {//Triangle button
                selectedServoName = "GripperRotator";
                selectedServo = initalizeServo(selectedServoName);
                selectedServo.setDirection(Servo.Direction.REVERSE);
                telemetry.addData("Selected Servo", selectedServoName);
                telemetry.update();
                sleep(300);
            }


        }

        //Display active control instructions
        telemetry.addLine("Control the selected servo using D-pad.")
                .addData("D-pad Up", "+0.1")
                .addData("D-pad Down", "-0.1")
                .addData("D-pad Right", "+0.01")
                .addData("D-pad Left", "-0.01")
                .addData("Current Position", String.format("%2f", servoPosition));
        telemetry.update();


        selectedServo.setPosition(servoPosition);

//TeleOp Phase: Control the selected servo
        while (opModeIsActive()) {
            updated = false;//Flag to check if servo position was updated
            //Check if the D-pad Up button is pressed
            if (gamepad1.dpad_up) {
                //Increase servo position by 0.1
                servoPosition += 0.1;
                //Clamp the servo position to a max of 1.0
                if (servoPosition > 1.0) {
                    servoPosition = 1.0;
                    telemetry.addLine("Servo position at maximum(1.0)");
                }
                updated = true;
                //add a short delay
                sleep(200);
            }

            //Check if D-pad Down button is pressed
            if (gamepad1.dpad_down) {
                //Decrease servo position by 0.1
                servoPosition -= 0.1;
                //Clamp the servo position to a minimum of 0.0
                if (servoPosition < 0.0) {
                    servoPosition = 0.0;
                    telemetry.addLine("Servo position at minimum(0.0)");
                }
                updated = true;
                //Add a short delay
                sleep(200);
            }
            //Check if D-pad Right button is pressed
            if (gamepad1.dpad_right) {
                //Increase servo position by 0.01
                servoPosition += 0.01;
                //Clamp the servo position to a max of 1.0
                if (servoPosition > 1.0) {
                    servoPosition = 1.0;
                    telemetry.addLine("Servo position at maximum(1.0)");
                }
                updated = true;
                //Add a short delay
                sleep(100);
            }

            //Check if the D-pad Left button is pressed
            if (gamepad1.dpad_left) {
                //Decrease servo position by 0.01
                servoPosition -= 0.01;
                //Clamp the servo position to a minimum of 0.0
                if (servoPosition < 0.0) {
                    servoPosition = 0.0;
                    telemetry.addLine("Servo position at minimum(0.0)");
                }
                updated = true;
                //Add a short delay
                sleep(100);
            }

//If servo position was updated,apply the new position
            if (updated) {
                telemetry.addData("In Update", "");

                try {
                    selectedServo.setPosition(servoPosition);
                    telemetry.addData("Setting Position", "");
                } catch (Exception e) {
                    telemetry.addData("Error", "Failed to set servo position: " + e.getMessage());
                }

                //Update the telemetry with the new servo position
                telemetry.addData("Selected Servo", selectedServoName);
                telemetry.addData("Servo Position", String.format("%2f", servoPosition));
            }
            //Small delay to improve loop performance
            telemetry.update();
            sleep(50);
        }
    }

    /*
    Initialize a servo and verifies its configuration.

        @param servoName The name of the servo as configured on the robot Controller App.
        @return True if the servo was successfully initialized, false otherwise.


     */
    private Servo initalizeServo(String servoName) {
        try {
            Servo servo = hardwareMap.get(Servo.class, servoName);
            if (servo == null) {
                telemetry.addData("Error", "Servo" + servoName + "not found.Please check configuration");
                telemetry.update();
                requestOpModeStop();
                return null;
            }
            telemetry.addData("Servo Initialized", servoName);
            telemetry.update();
            return servo;
        } catch (Exception e) {
            telemetry.addData("Error", "Exception initializing servo" + servoName + ":" + e.getMessage());
            telemetry.update();
            requestOpModeStop();
            return null;
        }
    }


}