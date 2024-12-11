package org.firstinspires.ftc.teamcode.intothedeep.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.intothedeep.Megalodog;


// The 'extends LinearOpMode' is needed so this code can run the build in op mode code from FIRST.
//    @Autonomous puts this code in the autonomous category on driver station

@Autonomous(name="Two Specimen and Park (no extra HH)", group="Autonomous")
public class NewOneSpecimenPushThree extends LinearOpMode {

    /* declare variables

     */
    @Override
    public void runOpMode() {

        // this lets us see how long the op mode has run

        Megalodog myBot = new Megalodog(this);
        myBot.InitializeDevices();
        // Put code that should run during initialization HERE in this area

        // Wait for the start button to be pressed on the driver station
        waitForStart();

        if (opModeIsActive()) {
            myBot.InitializePositions(false, true);

            // Go hang first specimen
            myBot.RaiseLift (Megalodog.liftUpperSpecimenBar,900);
            myBot.MoveStraight(-684,.20,500);
            //myBot.MoveStraight(-80,.10,500);
            myBot.MoveUntilButton(.4,-80);
            sleep(200);
            myBot.HookAndLetGo(Megalodog.liftPullSpecimenFromUpperBar, 700, 500);


            // hooked, now move back, over, and get around a sample ready to push
            myBot.MoveStraight(100,.25 ,200);
            myBot.RaiseLift(Megalodog.liftHome, 100);
            myBot.PutGripperAway();
            myBot.StrafeLeft(460, 0.35, 200);
            myBot.RotateLeft(185,.35,200);
            myBot.MoveStraight(610,.35, 200);
            myBot.StrafeRight(230,.25, 200);

            // now we're at a sample ready to push to human helper
            myBot.RotateRight(7, .4, 200);
            myBot.MoveStraight(-1040, .4, 200);

            // go back and get 2nd
            myBot.MoveStraight(1040, .4, 200);
            myBot.StrafeRight(200, .4, 200);
         //   myBot.RotateRight(5, .4, 200);
            myBot.OpenGripperForWall(0);

            // push to wall and grab next
            myBot.MoveStraight(-1040, .4, 200);
            myBot.MoveStraight(-90, .12, 400);
            myBot.MoveStraight(30, .12, 200);
            myBot.GrabSpecimenAndLift(500);  // cannot lower wait
            myBot.LevelGripper(0);

            // Go place HH specimen
            myBot.MoveStraight(250, .25, 200);  // 734 to bar
         //   myBot.RotateLeft(5, .4, 200);
            myBot.StrafeLeft(960, .4, 200);  // 840 to center
            myBot.RaiseLift(Megalodog.liftUpperSpecimenBar,100);
            myBot.RotateLeft(176,.45, 200);
            myBot.MoveStraight(-250,.18,200);  // was -705
            myBot.MoveUntilButton(.4,-90);
            myBot.HookAndLetGo(Megalodog.liftPullSpecimenFromUpperBar, 1050, 200);


            // Go Park
            myBot.MoveStraight(420,.4 ,100);
            myBot.DeployExtensionServo(200);
            myBot.RotateRight(3,.4, 100);
            myBot.PutGripperAway();
            myBot.RaiseLift(Megalodog.liftHome, 200);
            myBot.StrafeLeft(1300, 0.7, 100);
            myBot.RotateRight(180, .7, 100);
            myBot.MoveStraight(-80, .7, 2000);

    //        myBot.MoveStraight(1080, .4, 200);
    //        myBot.StrafeRight(150, .4, 200);
    //        myBot.MoveStraight(-1040, .4, 200);



            // Park
          //  myBot.MoveStraight(200,.25 ,200);
          //  myBot.DeployExtensionServo(400);
          //  myBot.RaiseLift(Megalodog.liftHome, 500);
          //  myBot.PutGripperAway();
          //  myBot.StrafeLeft(1180, 0.5, 1000);

            /*
            // hooked, now move back, over, and get around a sample ready to push
            myBot.MoveStraight(100,.25 ,200);
            myBot.RaiseLift(Megalodog.liftHome, 500);
            myBot.PutGripperAway();
            myBot.StrafeLeft(620, 0.25, 1000);
            myBot.RotateLeft(185,.25,1000);
            myBot.MoveStraight(620,.25, 800);
            myBot.StrafeRight(250,.25, 800);

            // now we're at a sample ready to push to human helper
            myBot.MoveStraight(-1020, .25, 1300);
            myBot.DeployAndOpenSpecimenGripper(0);


            // we delivered the sample, now move out of zone, wait, move in and grab specimen
            myBot.MoveStraight(240, .3, 700);
            myBot.StrafeLeft(280,.20, 700);
            myBot.RotateLeft(7,.2, 2600);
            myBot.MoveStraight(-350, .15, 1000);
            myBot.GrabSpecimenAndLift(700);

            // now go place the specimen
            myBot.MoveStraight(480, .3, 700);  // 734 to bar
            myBot.StrafeLeft(840, .2, 1200);  // 840 to center
            myBot.RotateLeft(184,.2, 1200);
            myBot.RaiseLift (Megalodog.liftUpperSpecimenBar,100);
            myBot.MoveStraight(58, .1, 400);
            myBot.HookAndLetGo(Megalodog.liftUpperSpecimenBar-1000, 700, 500);

            // now go park
            myBot.MoveStraight(200,.25 ,200);
            myBot.DeployExtensionServo(0);
            myBot.RaiseLift(Megalodog.liftHome, 500);
            myBot.PutGripperAway();
            myBot.StrafeLeft(1180, 0.25, 1000);
        */
        }
    }


}