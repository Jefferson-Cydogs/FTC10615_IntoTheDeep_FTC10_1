package org.firstinspires.ftc.teamcode.intothedeep.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.chassis.MegalodogChassis;
import org.firstinspires.ftc.teamcode.intothedeep.Megalodog;


// The 'extends LinearOpMode' is needed so this code can run the build in op mode code from FIRST.
//    @Autonomous puts this code in the autonomous category on driver station
@Disabled
@Autonomous(name="Right One Specimen", group="Autonomous")
public class RedRightOneSpecimen extends LinearOpMode {

    /* declare variables

     */
    @Override
    public void runOpMode() {

        // this lets us see how long the op mode has run

        Megalodog myBot = new Megalodog(this);
        myBot.InitializeDevices();
        myBot.SpecimenGripperServo.setPosition(Megalodog.specimenServoClosed);
        // Put code that should run during initialization HERE in this area

        // Wait for the start button to be pressed on the driver station
        waitForStart();

        if (opModeIsActive()) {
            myBot.InitializePositions(false, true);

            myBot.RaiseLift (Megalodog.liftUpperSpecimenBar+370,900);

            // 1: Move forward
            myBot.MoveStraight(-795,.12,2000);  // was -705
            myBot.MoveStraight(51,.12,500);
            // 2: Hang specimen
            myBot.HookAndLetGo(Megalodog.liftUpperSpecimenBar-1000, 1050, 500);
            // 3: Move backwards
            myBot.MoveStraight(495,.25 ,200);
            myBot.RaiseLift(Megalodog.liftHome, 500);
            myBot.PutGripperAway();
            // 4: Strafe Right
             myBot.StrafeLeft(1180, 0.25, 1000);
            // Advanced (from where we drop specimen): Strafe right, grab first sample (closest one)
            // Rotate right, strafe left, drop sample, move backwards
            myBot.RotateLeft(184,.3,800);
            myBot.RaiseLift(Megalodog.liftLowerBasket,2000);
            myBot.DeployExtensionServo(300);
            myBot.RaiseLift(Megalodog.liftHome,1500);

        }
    }


}