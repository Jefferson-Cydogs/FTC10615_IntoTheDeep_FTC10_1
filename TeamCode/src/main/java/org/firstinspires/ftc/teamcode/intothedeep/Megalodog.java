package org.firstinspires.ftc.teamcode.intothedeep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.chassis.MegalodogChassis;


public class Megalodog {
    public int top_bucket=2000;
    public int low_bucket=1000;
    public int ArmLinearSlide;
    public int intakeExtension;
    public int Lift;
    public int grabSpeiceman;
    public int hangSpeciceman;
    public int ReturnLift;


    public void runintake(int direction,int howLong,int wait){
        //turns servo for a certain amount of time
    }
    public void RealeaseIntoBucket(int wait){
        //spin servo for a short amount of time
    }
    public void Returnlift(int wait){
        //turn motor in opposite direction for a certain amount of time or until it hits the presher senser
    }
    public void GrabandLift(int height, int wait){
        //turn servo that grabs the speiceman and raises lift to a certain height
    }

    public int ArmGround;
    public int SampleLow;
    public int SampleHigh;

    public int LinearSlideFar;
    public int LinearSlideHome;

    public int SpecimenHigh;
    public int SpecimenLow;

    public int SpecimenGripperClosed;
    public int SpecimenGripperOpen;
    //------------------------------------------------------------------------
    public void MoveSlideAndScoop (int distanceMM,int wait){};
//the motor makes the slide move to a certain distance and the the scoop goes down
    public void RaiseLift (int hightMM, int wait){};
//raises the lift to a certain height
    public void EmptyLift (int wait){};
//servo lifts and empties the bucket
    public void LetGoOfSpecimen(int wait){};
//servo moves in reverse to let go of the speiceman
    public void CheckSampleColor (){};
//the camera checks the color of the sample inside the scoop
    private int topBucket = 2000;
    private int lowBucket = 1000;
    private int highBar = 1500;
    private int lowBar = 500;
    private double Gripper = 105.8;
    private int highChamber = 1200;
    private int lowChamber = 255;
    private int deliveryBox = 105;
    private Servo Intake;


    public void TurnIntakeOn () {
//It turns the continues servo forward

    }
    public void TurnIntakeOff () {
//It turns the continues servo off

    }
    public void GrabSpeicen (int waitime) {
//The servo rotates forward to grag the speiceman

    }
    public void HookAndLetGo (int waitime) {
// It pushes the sepiecem that is in the gripper down and then lets go of it

    }
}

