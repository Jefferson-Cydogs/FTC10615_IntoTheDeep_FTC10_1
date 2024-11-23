package org.firstinspires.ftc.teamcode.intothedeep.autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.teamcode.chassis.MegalodogChassis;
import org.firstinspires.ftc.teamcode.intothedeep.Megalodog;


// The 'extends LinearOpMode' is needed so this code can run the build in op mode code from FIRST.
//    @Autonomous puts this code in the autonomous category on driver station
@Autonomous(name="Left Samples", group="Autonomous", preselectTeleOp = "The Final Countdown")
public class RedLeftOneSample extends LinearOpMode {

    /* declare variables

     */
    @Override
    public void runOpMode() {

        double autonSpeed = 0.35;
        int deliveryBoxDumpWaitTime = 900;

        Megalodog myBot = new Megalodog(this);
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
            myBot.MoveStraight(-470,.25, 200); // was 1200, velocity .2
            myBot.DumpDeliveryBox(deliveryBoxDumpWaitTime);
            myBot.MoveStraight(210,autonSpeed,100);
            myBot.LiftMoveToHome(200);  // changed from 600 in good run//  450 last time

            // 2nd sample
            myBot.StrafeLeft(50,autonSpeed,200);
            myBot.RotateLeft(94,autonSpeed,200); // was 300
            myBot.TurnIntakeOn(.22);
            myBot.MoveStraight(390,autonSpeed,100); // was 200 wait


            // testing adjustment
            myBot.RotateLeft(12, .5, 100); // was 150

            myBot.ScootAndScoop(50, 200); // was 230 wait
            myBot.ExchangeServoRaise(100);
            myBot.RotateRight(36,autonSpeed,200); // changed from 20

            myBot.MoveStraight(-485,.3,100);
            myBot.ExchangeSample(0);

          //  myBot.RotateRight(22,autonSpeed,100);
         //   myBot.RaiseLift(Megalodog.liftUpperBasket,1700);
            myBot.RaiseLift(Megalodog.liftUpperBasket,100);
            myBot.RotateRight(27,autonSpeed,900);


            myBot.DumpDeliveryBox(deliveryBoxDumpWaitTime);
            myBot.MoveStraight(110,autonSpeed,200);
            myBot.LiftMoveToHome(300); // was 1000

            // third sample
            myBot.RotateLeft(67.5,autonSpeed,200);
            myBot.MoveStraight(220,.4,300);

            // testing adjustment
            //myBot.RotateLeft(15, .5, 150);

            myBot.ScootAndScoop(50, 200);  // was 240 wait
            myBot.ExchangeServoRaise(100);
            myBot.RotateRight(38,autonSpeed,100);  // was 35
            myBot.StrafeRight(55,autonSpeed,300); // was 62

            //This was the exchange sample function, built the move straight into it
            myBot.IntakeBoxServo.setPower(-.14);  // was -.18
            myBot.MoveStraight(-405,autonSpeed,200);
            myBot.IntakeBoxServo.setPower(0);
            myBot.ExtensionServo.setPosition(Megalodog.extensionServoSafetyPosition);

            myBot.RaiseLift(Megalodog.liftUpperBasket,1400); // was 2200
            myBot.DumpDeliveryBox(deliveryBoxDumpWaitTime);
            myBot.LiftMoveToHome(500);  //was 1000

            // fourth sample!
            myBot.RotateLeft(41, autonSpeed, 100);
            myBot.MoveStraight(225, autonSpeed, 100);
            myBot.ScootAndScoop(50, 220);  // was 240 wait
            myBot.ExchangeServoRaise(100);
            myBot.RotateRight(19, autonSpeed, 100);
            myBot.ExchangeSample(0);
            myBot.MoveStraight(-350, autonSpeed, 100);

            myBot.RaiseLift(Megalodog.liftUpperBasket,1100);
            myBot.RotateRight(8, autonSpeed, 100);

            myBot.DumpDeliveryBox(deliveryBoxDumpWaitTime);
            myBot.LiftMoveToHome(500);  //was 1000
            sleep(2000);

          //  sleep(1000);


        }
    }


}


