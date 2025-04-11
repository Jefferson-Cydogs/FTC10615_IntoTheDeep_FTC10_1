package org.firstinspires.ftc.teamcode.intothedeep.autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.intothedeep.Megalodog;


// The 'extends LinearOpMode' is needed so this code can run the build in op mode code from FIRST.
//    @Autonomous puts this code in the autonomous category on driver station
@Disabled
@Autonomous(name="Left Samples 2", group="Autonomous", preselectTeleOp = "Shark Attack!")
public class RedLeftOneSampleV2 extends LinearOpMode {

    /* declare variables

     */
    private Megalodog myBot;
    @Override
    public void runOpMode() {

        double autonSpeed = 0.35;
        int deliveryBoxDumpWaitTime = 900;

        myBot = new Megalodog(this);
        myBot.InitializeDevices();
        // Put code that should run during initialization HERE in this area
        telemetry.addLine("It's okay to start now");
        telemetry.update();
        // Wait for the start button to be pressed on the driver station
        waitForStart();

        if (opModeIsActive()) {

            // 1st sample
            myBot.InitializePositions(true, false);
            myBot.RaiseLift(Megalodog.liftUpperBasket,300);
            myBot.MoveStraight(-485,.25, 200); // was 1200, velocity .2
            myBot.DumpDeliveryBox(deliveryBoxDumpWaitTime);
            myBot.MoveStraight(250,autonSpeed,100);
            //above was 210
            myBot.LiftMoveToHome(200);  // changed from 600 in good run//  450 last time

    // 2nd sample
            myBot.StrafeLeft(53,autonSpeed,200);  // was 50
            myBot.RotateLeft(97,autonSpeed,200); // was 300
            myBot.TurnIntakeOn(.2);
            myBot.MoveStraight(350,autonSpeed,100); // was 200 wait

            // testing adjustment
            myBot.RotateLeft(12, .5, 100); // was 150

            // SCOOT AND SCOOP
            myBot.ExtensionServo.setPosition(Megalodog.extensionServoFloor);
            myBot.MoveStraight(230,.2,100);
            pressDowntoGrab(1100);
            myBot.TurnIntakeOn(.08);
            myBot.RotateRight(38,autonSpeed,100); // changed from 20


            // these happen at same time
            myBot.ExchangeServoRaise(100);
            myBot.ExtensionBoxRotation.setPosition(Megalodog.extensionBoxRotatorDumping);
            myBot.MoveStraight(-475,.3,100);
            myBot.TurnIntakeOff();
            myBot.IntakeBoxServo.setPower(-.29);  // was -.18
            sleep(700); // was 1000
            //myBot.IntakeBoxServo.setPower(0);
            //sleep(100);
            //myBot.IntakeBoxServo.setPower(.18);
            //sleep(600);
            myBot.IntakeBoxServo.setPower(0);
            myBot.ExtensionBoxRotation.setPosition(Megalodog.extensionBoxRotatorStarting);
            myBot.ExtensionServo.setPosition(Megalodog.extensionServoSafetyPosition);

          //  myBot.RotateRight(22,autonSpeed,100);
         //   myBot.RaiseLift(Megalodog.liftUpperBasket,1700);
            myBot.RaiseLift(Megalodog.liftUpperBasket,100);
            myBot.RotateRight(27,autonSpeed,900);


            myBot.DumpDeliveryBox(deliveryBoxDumpWaitTime);
            myBot.MoveStraight(110,autonSpeed,200);
            myBot.LiftMoveToHome(300); // was 1000

    // third sample
            myBot.RotateLeft(64,autonSpeed,200);
            myBot.MoveStraight(140,.4,300);

            // testing adjustment
            //myBot.RotateLeft(15, .5, 150);

            //myBot.ScootAndScoop(50, 200);  // was 240 wait

            // SCOOT AND SCOOP
            myBot.ExtensionServo.setPosition(Megalodog.extensionServoFloor);
            myBot.TurnIntakeOn(.2);
            //myOpMode.sleep(100); // was 200
            myBot.MoveStraight(200,.25,100);

            pressDowntoGrab(1100);

            myBot.TurnIntakeOn(.05);
            myBot.ExchangeServoRaise(100);
            myBot.TurnIntakeOff();
            myBot.RotateRight(40,autonSpeed,100);  // was 35
            myBot.ExtensionBoxRotation.setPosition(Megalodog.extensionBoxRotatorDumping);
            myBot.StrafeRight(150,autonSpeed,100); // was 62


            //This was the exchange sample function, built the move straight into it
            myBot.IntakeBoxServo.setPower(-.29);  // was -.18
            myBot.MoveStraight(-405,autonSpeed,400);
            myBot.RotateRight(15, .35, 100);
            //myBot.IntakeBoxServo.setPower(0);
            //sleep(100);
            //myBot.IntakeBoxServo.setPower(.17);
            //sleep(600);
            myBot.IntakeBoxServo.setPower(0);
            myBot.ExtensionServo.setPosition(Megalodog.extensionServoSafetyPosition);
            myBot.ExtensionBoxRotation.setPosition(Megalodog.extensionBoxRotatorStarting);

            myBot.RaiseLift(Megalodog.liftUpperBasket,1400); // was 2200
            myBot.DumpDeliveryBox(deliveryBoxDumpWaitTime);
            myBot.LiftMoveToHome(500);  //was 1000

    // fourth sample!
            myBot.MoveStraight(320, .4, 100);
            myBot.RotateLeft(98, .4, 100);
            myBot.MoveStraight(20, autonSpeed, 100);

           // myBot.ScootAndScoop(50, 220);  // was 240 wait
            myBot.ExtensionServo.setPosition(Megalodog.extensionServoFloor);
            myBot.TurnIntakeOn(.20);
            //myOpMode.sleep(100); // was 200
            myBot.MoveStraight(320,.25,100);
            pressDowntoGrab(1000);
            myBot.TurnIntakeOn(.08);
            myBot.MoveStraight(-50,.4,100);
            myBot.TurnIntakeOff();


            myBot.ExchangeServoRaise(600);
            myBot.ExtensionBoxRotation.setPosition(Megalodog.extensionBoxRotatorDumping);
            myBot.RotateRight(71, .3, 100);

            myBot.IntakeBoxServo.setPower(-.29);  // was -.18
            //sleep(100); // was 1000
            //myBot.IntakeBoxServo.setPower(.18);  // was -.18
            //sleep(700); // was 1000

            myBot.MoveStraight(-405, .4, 100);
            myBot.IntakeBoxServo.setPower(0);
            myBot.ExtensionServo.setPosition(Megalodog.extensionServoSafetyPosition);
            myBot.ExtensionBoxRotation.setPosition(Megalodog.extensionBoxRotatorStarting);

            myBot.RaiseLift(Megalodog.liftUpperBasket,1000);
            //myBot.RotateRight(8, autonSpeed, 100); // was 18
            //myBot.RotateLeft(8,autonSpeed, 100);

            myBot.DumpDeliveryBox(deliveryBoxDumpWaitTime);
            myBot.LiftMoveToHome(500);  //was 1000
            sleep(2000);

          //  sleep(1000);


        }
    }

    private void pressDowntoGrab(int intakeWaitTime)
    {
        myBot.ExtensionServo.setPosition(Megalodog.extensionServoFloor+0.125); // was -0.018
        myBot.ExtensionBoxRotation.setPosition(Megalodog.extensionBoxRotatorStarting);
        sleep(intakeWaitTime);
        myBot.ExtensionServo.setPosition(Megalodog.extensionServoFloor);
        myBot.ExtensionBoxRotation.setPosition(Megalodog.extensionBoxRotatorStarting);
    }

}


