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
        int deliveryBoxDumpWaitTime = 1100;

        // this lets us see how long the op mode has run

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
            myBot.RaiseLift(Megalodog.liftUpperBasket,1300);
            myBot.MoveStraight(-325,.2, 200);
            myBot.DumpDeliveryBox(deliveryBoxDumpWaitTime);
            myBot.MoveStraight(210,autonSpeed,100);  // was 85
            myBot.LiftMoveToHome(450);  // changed from 600 in good run

            // 2nd sample
            myBot.StrafeLeft(50,autonSpeed,200);
            myBot.RotateLeft(92,autonSpeed,300);
            myBot.MoveStraight(335,autonSpeed,300);


            // testing adjustment
            myBot.RotateLeft(10, .5, 150);

            myBot.ScootAndScoop(200, 230);
            myBot.RotateRight(36,autonSpeed,200); // changed from 20
            myBot.ExchangeServoRaise(200);
            myBot.MoveStraight(-480,.4,100);
            myBot.ExchangeSample(0);
            myBot.RotateRight(22,autonSpeed,100);
      //      myBot.StrafeRight(20,autonSpeed,100);
            myBot.RaiseLift(Megalodog.liftUpperBasket,1700);
            myBot.DumpDeliveryBox(deliveryBoxDumpWaitTime);
            myBot.MoveStraight(115,autonSpeed,200);
            myBot.LiftMoveToHome(1000);

            // third sample
            myBot.RotateLeft(49,autonSpeed,200);
            myBot.MoveStraight(185,.4,300);

            // testing adjustment
            myBot.RotateLeft(15, .5, 150);

            myBot.ScootAndScoop(200, 240);
            myBot.RotateRight(39,autonSpeed,100);  // was 35
            myBot.ExchangeServoRaise(200);
            myBot.StrafeRight(62,autonSpeed,300);
            myBot.MoveStraight(-420,autonSpeed,0);
            myBot.ExchangeSample(100);
            myBot.RaiseLift(Megalodog.liftUpperBasket,2200);
            myBot.DumpDeliveryBox(deliveryBoxDumpWaitTime);
            myBot.LiftMoveToHome(1000);
            sleep(2000);

          //  sleep(1000);


        }
    }


}


