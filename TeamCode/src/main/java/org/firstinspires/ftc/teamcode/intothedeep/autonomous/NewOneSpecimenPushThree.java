package org.firstinspires.ftc.teamcode.intothedeep.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.intothedeep.Megalodog;


// The 'extends LinearOpMode' is needed so this code can run the build in op mode code from FIRST.
//    @Autonomous puts this code in the autonomous category on driver station

@Autonomous(name="Red Two Specimen (no extra HH)", group="Autonomous", preselectTeleOp = "MEG Attack! Red")
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
            myBot.RaiseLift (Megalodog.liftUpperSpecimenBar,200);  // was 900
            myBot.MoveStraight(-684,.20,200); // was 200
            //myBot.MoveStraight(-80,.10,500);
            if(!myBot.MoveUntilButton(.4,-80))
            {
                myBot.MoveStraight(50,.3,100);
            }
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
            myBot.MoveStraight(-100, .12, 200);  //was 400
            myBot.MoveStraight(30, .12, 200);
            myBot.GrabSpecimenAndLift(500);  // cannot lower wait
            myBot.LevelGripper(0);

            // Go place HH specimen
            myBot.MoveStraight(250, .25, 200);  // 734 to bar
         //   myBot.RotateLeft(5, .4, 200);
            myBot.StrafeLeft(1100, .4, 200);  // 840 to center
            myBot.RaiseLift(Megalodog.liftUpperSpecimenBar,100);
            myBot.RotateLeft(176,.45, 200);
            myBot.MoveStraight(-300,.18,200);  // was -705
            if(!myBot.MoveUntilButton(.4,-80))
            {
                myBot.MoveStraight(50,.3,100);
            }
            myBot.HookAndLetGo(Megalodog.liftPullSpecimenFromUpperBar, 1050, 200);


            // Go Park
            myBot.MoveStraight(370,.4 ,100);
            myBot.DeployExtensionServo(200);
            myBot.RotateRight(3,.4, 100);
            myBot.PutGripperAway();
            myBot.RaiseLift(Megalodog.liftHome, 200);
            myBot.StrafeLeft(1220, 0.7, 100);
            myBot.RotateRight(180, .7, 100);
            myBot.MoveStraight(-80, .7, 2000);

        }
    }


}