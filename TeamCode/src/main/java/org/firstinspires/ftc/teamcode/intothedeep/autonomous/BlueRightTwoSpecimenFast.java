package org.firstinspires.ftc.teamcode.intothedeep.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.intothedeep.Megalodog;


// The 'extends LinearOpMode' is needed so this code can run the build in op mode code from FIRST.
//    @Autonomous puts this code in the autonomous category on driver station
@Disabled
@Autonomous(name="Right Two Specimen / Park", group="Autonomous", preselectTeleOp = "The Final Countdown")
public class BlueRightTwoSpecimenFast extends LinearOpMode {

    /* declare variables

     */
    @Override
    public void runOpMode() {

        // this lets us see how long the op mode has run

        Megalodog myBot = new Megalodog(this);
        myBot.InitializeDevices();
        myBot.SpecimenGripperServo.setPosition(Megalodog.specimenServoClosed);
        telemetry.addLine("It's okay to start now");
        telemetry.update();
        // Put code that should run during initialization HERE in this area

        // Wait for the start button to be pressed on the driver station
        waitForStart();

        if (opModeIsActive()) {
            myBot.InitializePositions(false, true);

            // Go hang first specimen
            myBot.RaiseLift (Megalodog.liftUpperSpecimenBar+370,100);
         //   myBot.MoveStraight(-633,.25,800);
         //   myBot.MoveStraight(-80,.10,500);
            myBot.MoveStraight(-795,.14,200);  // was -705
            myBot.MoveStraight(66,.12,200);
            myBot.HookAndLetGo(Megalodog.liftUpperSpecimenBar-1050, 1100, 0);

            // hooked, now move back, over, and get around a sample ready to push
            myBot.MoveStraight(100,.25 ,200);
            myBot.RaiseLift(Megalodog.liftHome, 100);
            myBot.PutGripperAway();
            myBot.StrafeLeft(460, 0.25, 400);
            myBot.RotateLeft(185,.25,300);
            myBot.MoveStraight(610,.3, 200);
            myBot.StrafeRight(240,.25, 200);

            // now we're at a sample ready to push to human helper
            myBot.MoveStraight(-1040, .4, 200);
            myBot.DeployAndOpenSpecimenGripper(0);


            // we delivered the sample, now move out of zone, wait, move in and grab specimen
            myBot.MoveStraight(240, .35, 1900);
            //myBot.StrafeLeft(280,.20, 1500);
            //myBot.RotateLeft(7,.2, 1600);
            myBot.MoveStraight(-310, .13, 200);
            myBot.GrabSpecimenAndLift(500);  // cannot lower wait

            // now go place the specimen
            myBot.MoveStraight(250, .25, 200);  // 734 to bar
            myBot.StrafeLeft(1120, .35, 200);  // 840 to center
            myBot.RaiseLift (Megalodog.liftUpperSpecimenBar+370,100);
            myBot.RotateLeft(184,.45, 200);

            //myBot.MoveStraight(-280, .2, 800);
            myBot.MoveStraight(-470,.18,200);  // was -705
            myBot.MoveStraight(100,.15,200);
            myBot.HookAndLetGo(Megalodog.liftUpperSpecimenBar-1000, 1050, 200);


            // end here
            myBot.MoveStraight(100,0.7,0);
            myBot.PutGripperAway();
            myBot.DeployExtensionServo(300);
            myBot.StrafeLeft(1200, 0.7, 200);
            myBot.RaiseLift(Megalodog.liftHome, 0);
            myBot.MoveStraight(320,0.7,200);

            // now go park
            //myBot.MoveStraight(300,.25 ,200);
            //myBot.DeployExtensionServo(0);
            //myBot.RaiseLift(Megalodog.liftHome, 100);
            //myBot.PutGripperAway();
            //myBot.StrafeLeft(1180, 0.25, 1000);

        }
    }


}