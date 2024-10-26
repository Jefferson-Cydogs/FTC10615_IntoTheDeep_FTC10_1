package org.firstinspires.ftc.teamcode.intothedeep.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.chassis.MegalodogChassis;
import org.firstinspires.ftc.teamcode.intothedeep.Megalodog;


// The 'extends LinearOpMode' is needed so this code can run the build in op mode code from FIRST.
//    @Autonomous puts this code in the autonomous category on driver station
@Autonomous
public class RedRightOneSpecimen extends LinearOpMode {

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
            //            // Put code that sould run during the active mode HERE in this area
            //myBot.InitializePositions();
            myBot.RaiseLift (Megalodog.liftUpperSpecimenBar,900);

            // 1: Move forward
            myBot.MoveStraight(-730,.20,500);
            // 2: Hang specimen
            sleep(2000);
            myBot.HookAndLetGo(Megalodog.liftUpperSpecimenBar-1000, 1000);
            // 3: Move backwards
            myBot.MoveStraight(495,.25 ,200);
            myBot.RaiseLift(Megalodog.liftHome, 500);
            // 4: Strafe Right
             myBot.StrafeLeft(1160, 0.25, 1000);
            // Advanced (from where we drop specimen): Strafe right, grab first sample (closest one)
            // Rotate right, strafe left, drop sample, move backwards
            myBot.RotateLeft(180,.5,500);

        }
    }


}