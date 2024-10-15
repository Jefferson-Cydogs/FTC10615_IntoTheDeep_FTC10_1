package org.firstinspires.ftc.teamcode.configs;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class Config_Motor extends LinearOpMode {

    //Declare the motors
    private DcMotor IntakeExtension;
    private DcMotor Lift;
    private DcMotor motor3;
    private DcMotor motor4;

    //variable to keep track of the selected motor
    private  DcMotor selectedMotor = null;
    private String selectedMotorName = "None";

    //Define the inital motor target position

    private int motorTargetPosition = 0;

    //Timer for optional timeout in motor selection
    private ElapsedTime selectionTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        //Initilize the motors with error handling

        IntakeExtension = initializeMotor("RedSlide");
        Lift = initializeMotor("BlueSlide");
        motor3 = initializeMotor("FrontLeftWheel");
        motor4 = initializeMotor("FrontRightWheel");

        //set inital taget positions to 0

        IntakeExtension.setTargetPosition(0);
        Lift.setTargetPosition(0);
        motor3.setTargetPosition(0);
        motor4.setTargetPosition(0);

        //Reset encoders and set motors to RUN_TO_POSITION mode
        resetAndConfigureMotor(IntakeExtension);
        resetAndConfigureMotor(Lift);
        resetAndConfigureMotor(motor3);
        resetAndConfigureMotor(motor4);

        IntakeExtension.setDirection(DcMotorSimple.Direction.REVERSE);
        Lift.setDirection(DcMotorSimple.Direction.REVERSE);

        //Set initial motor power to 0 (no movement)
        IntakeExtension.setPower(0);
        Lift.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);


//Wati for the game to start(driver presses PLAY
        waitForStart();

        //Display selection instructions during initialization
        telemetry.addLine("Select Motor to Test using PlayStation Controller Buttons.")
                .addData("Cross(X)","Red Slide")
                .addData("Circle(O)","Blue Slide")
                .addData("Square()","Left Front Wheel")
                .addData("Triangle()","Right Front Wheel")
                .addData("Current Selection",selectedMotorName);
        telemetry.update();


        while (opModeIsActive()&&selectedMotor == null){
            //Check for motor selection buttons based on PlayStation controller mapping
            if(gamepad1.a) {//Cross() button
                selectedMotor = IntakeExtension;
                selectedMotorName = "Motor 1";
                motorTargetPosition = 0; //Start at position 0
                selectedMotor.setTargetPosition(motorTargetPosition);
                telemetry.addData("Selected Motor", selectedMotorName);
                telemetry.update();
                sleep(300);
            }else if (gamepad1.b) {//Circle(O)button
                selectedMotor = Lift;
                selectedMotorName = "Motor2";
                motorTargetPosition = 0; //Start position 0
                selectedMotor.setTargetPosition(motorTargetPosition);
                telemetry.addData("Selected Motor", selectedMotorName);
                telemetry.update();
                sleep(300);
            }else if (gamepad1.x){//square()button
                selectedMotor = motor3;
                selectedMotorName = "Motor3;";
                motorTargetPosition = 0;//Start at position 0
                selectedMotor.setTargetPosition(motorTargetPosition);
                telemetry.addData("Motor", selectedMotorName);
                telemetry.update();
                sleep(300);
            } else if (gamepad1.y) {//Triangle()button
                selectedMotor = motor4;
                selectedMotorName = "Motor 4";
                motorTargetPosition = 0; //Start at position 0
                selectedMotor.setTargetPosition(motorTargetPosition);
                telemetry.addData("Selected Motor", selectedMotorName);
                telemetry.update();
                sleep(300);
            }
            if (selectionTimer.seconds()>30){
                selectedMotor=IntakeExtension;
                selectedMotorName="Motor 1(default)";
            }

        }

        //If no motor was selected, default to Motor 1
        if(selectedMotor==null){
            selectedMotor=IntakeExtension;
            selectedMotorName="Motor 1 (Defualt)";
            motorTargetPosition=0;
            selectedMotor.setTargetPosition(motorTargetPosition);
            telemetry.addData("Selected MOtor",selectedMotorName);
            telemetry.update();
            sleep(300);
        }

        telemetry.addLine("Control the selected motor's position using D-pad:")
                .addData("D-pad Up","+100")
                .addData("D-pad Up","-100")
                .addData("D-pad Up","+10")
                .addData("D-pad Up","-10")
                .addData("Current Taget Position",motorTargetPosition);
        telemetry.update();
        while(opModeIsActive()){
            boolean updated=false;

            if (gamepad1.dpad_up) {
                motorTargetPosition += 100;
                if (motorTargetPosition > 3000) {
                    motorTargetPosition = 3000;
                    telemetry.addLine("Motor target position at maximum(3000)");
                }
                updated = true;
                sleep(300);
            }
            if(gamepad1.dpad_down){
                motorTargetPosition -= 100;
                if(motorTargetPosition<0){
                    motorTargetPosition=0;
                    telemetry.addLine("Motor target position at minimum(0)");
                }
                updated=true;
                sleep(300);
            }
            if(gamepad1.dpad_right){
                motorTargetPosition+=10;
                if(motorTargetPosition>3000){
                    motorTargetPosition = 3000;
                    telemetry.addLine("Motor target position at maximum(3000)");
                }
                updated=true;
                sleep(150);
            }
            if(gamepad1.dpad_left){
                motorTargetPosition-=10;
                if(motorTargetPosition<0){
                    motorTargetPosition=0;
                    telemetry.addLine("Motor target position at minimum (0)");
                }
                updated=true;
                sleep(150);
            }
            if(updated){
                try{
                    selectedMotor.setTargetPosition(motorTargetPosition);
                    selectedMotor.setPower(0.5);
                }catch(Exception e){
                    telemetry.addData("Error","Failed to set motor target position:"+e.getMessage());
                    telemetry.update();
                }
                telemetry.addData("Selecter Motor", motorTargetPosition);
                telemetry.addData("Target Position",motorTargetPosition);
                telemetry.update();
            }
            if(selectedMotor.isBusy()) {
                telemetry.addLine("Motor is moving to target position...");
            }
            else
            {
                telemetry.addLine("Motor reached target position");
            }
            telemetry.update();
            sleep(50);
        }
    }

    private DcMotor initializeMotor(String motorName) {
        try {
            DcMotor motor = hardwareMap.get(DcMotor.class, motorName);
            if (motor == null) {
                telemetry.addData("Error", "Motor" + motorName + "not found.Please c.heck configuration.");
                telemetry.update();
                requestOpModeStop();
                return null;
            }
            telemetry.addData("Motor Initialized", motorName);
            telemetry.update();
            return motor;
        } catch (Exception e) {
            telemetry.addData("Error", "Exception initilize motor" + motorName + ":" + e.getMessage());
            telemetry.update();
            return null;
        }
    }
    private void resetAndConfigureMotor(DcMotor motor){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}

