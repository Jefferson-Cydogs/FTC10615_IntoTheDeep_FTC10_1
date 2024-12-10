package org.firstinspires.ftc.teamcode.intothedeep.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.intothedeep.Megalodog;


// The 'extends LinearOpMode' is needed so this code can run the build in op mode code from FIRST.
//    @Autonomous puts this code in the autonomous category on driver station
@Autonomous(name="Right Specimen if HH has an extra", group="Autonomous", preselectTeleOp = "Shark Attack!")
public class NewOnePlusHHSpecimen extends LinearOpMode {

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
            myBot.MoveStraight(-654,.3,500);
            //myBot.MoveStraight(-80,.10,500);
            myBot.MoveUntilButton(.4,-75);
            sleep(300);
            myBot.HookAndLetGo(Megalodog.liftPullSpecimenFromUpperBar, 700, 500);

            // Go get HH Specimen
            myBot.MoveStraight(200,.25 ,200);
            myBot.RaiseLift(Megalodog.liftHome, 0);
            myBot.StrafeLeft(900, 0.5, 500);
            myBot.RotateLeft(180, .4, 300);
            // get specimen from wall
                myBot.MoveStraight(-420, .25, 100);
                myBot.OpenGripperForWall(0);
                myBot.MoveStraight(-130, .12, 700);

                myBot.MoveStraight(34, .12, 200);
                myBot.GrabSpecimenAndLift(500);  // cannot lower wait
                myBot.LevelGripper(0);
            myBot.GrabSpecimenAndLift(500);

            // Go place HH specimen
            myBot.MoveStraight(250, .25, 200);  // 734 to bar
            myBot.StrafeLeft(970, .35, 200);  // 840 to center
            myBot.RaiseLift(Megalodog.liftUpperSpecimenBar,100);
            myBot.RotateLeft(176,.45, 200);
            myBot.MoveStraight(-430,.18,200);  // was -705
            myBot.MoveUntilButton(.4,-135);
            myBot.HookAndLetGo(Megalodog.liftPullSpecimenFromUpperBar, 1050, 200);


            // Go Park
            myBot.MoveStraight(420,.4 ,200);
            myBot.DeployExtensionServo(200);
            myBot.RotateRight(3,.4, 200);
            myBot.PutGripperAway();
            myBot.RaiseLift(Megalodog.liftHome, 200);
            myBot.StrafeLeft(1300, 0.7, 300);
            myBot.RotateRight(180, .7, 300);
            myBot.MoveStraight(-220, .7, 2000);

        }
    }


}