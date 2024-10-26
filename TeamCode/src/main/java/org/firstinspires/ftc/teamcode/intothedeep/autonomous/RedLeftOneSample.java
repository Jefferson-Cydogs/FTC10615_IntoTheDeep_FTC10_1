package org.firstinspires.ftc.teamcode.intothedeep.autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.teamcode.chassis.MegalodogChassis;
import org.firstinspires.ftc.teamcode.intothedeep.Megalodog;


// The 'extends LinearOpMode' is needed so this code can run the build in op mode code from FIRST.
//    @Autonomous puts this code in the autonomous category on driver station
@Autonomous
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

        // Wait for the start button to be pressed on the driver station
        waitForStart();

        if (opModeIsActive()) {

            // 1st sample
            myBot.InitializePositions(true, false);
            myBot.RaiseLift(Megalodog.liftUpperBasket,1300);
            myBot.MoveStraight(-145,.2, 200);
            myBot.DumpDeliveryBox(deliveryBoxDumpWaitTime);
            myBot.MoveStraight(85,autonSpeed,100);
            myBot.LiftMoveToHome(600);

            // 2nd sample
            myBot.StrafeLeft(50,autonSpeed,200);
            myBot.RotateLeft(90,autonSpeed,300);
            myBot.MoveStraight(345,autonSpeed,300);
            myBot.ScootAndScoop(200);
            myBot.RotateRight(20,autonSpeed,200);
            myBot.ExchangeServoRaise(200);
            myBot.MoveStraight(-500,.4,100);
            myBot.ExchangeSample(0);
            myBot.RotateRight(22,autonSpeed,100);
            myBot.StrafeRight(62,autonSpeed,100);
            myBot.RaiseLift(Megalodog.liftUpperBasket,1700);
            myBot.DumpDeliveryBox(deliveryBoxDumpWaitTime);
            myBot.MoveStraight(115,autonSpeed,200);
            myBot.LiftMoveToHome(1000);

            // third sample
            myBot.RotateLeft(58,autonSpeed,200);
            myBot.MoveStraight(240,.4,300);
            myBot.ScootAndScoop(200);
            myBot.RotateRight(35,autonSpeed,100);
            myBot.ExchangeServoRaise(200);
            myBot.StrafeRight(50,autonSpeed,300);
            myBot.MoveStraight(-400,autonSpeed,0);
            myBot.ExchangeSample(100);
            myBot.RaiseLift(Megalodog.liftUpperBasket,2200);
            myBot.DumpDeliveryBox(deliveryBoxDumpWaitTime);
            myBot.LiftMoveToHome(1000);
            sleep(2000);

          //  sleep(1000);


        }
    }


}


