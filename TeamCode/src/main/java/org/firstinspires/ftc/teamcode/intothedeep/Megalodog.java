package org.firstinspires.ftc.teamcode.intothedeep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.chassis.MegalodogChassis;

public class Megalodog extends MegalodogChassis {
    public final static int extensionSliderMax = 1300;
    public final static int liftLowerBasket = 1900;
    public final static int liftHome = 30;
    public final static int liftUpperBasket = 4700;
    public final static int liftLowerSpecimenBar = 500;
    public final static int liftUpperSpecimenBar = 2600;
    public final static int liftPullSpecimenFromUpperBar = 1600;
    public final static int liftSnapSpecimen = 200;
    public final static int liftGetSpecimenFromWall = 500;
    public final static int liftHangOnLowerBar = 2000;
    public final static int liftHangOnUpperBar = 1000;
    // .89 was good when angled higher
    //  .93 was config servo at lower angle
    public final static double extensionServoFloor = 0.87;
    public final static double extensionServoDump = 0.20;
    public final static double deliveryServoHome = 0.35;
    public final static double deliveryServoDump = 0.94;
    public final static double specimenServoOpen = 0.21;
    public final static double specimenServoClosed = 0.45;
    public final static double specimenServoStarting = 0.4;
    public final static double continuousIntakePower = 0.4;
    public final static double gripperRotatorStarting = 0.53;
    public final static double gripperRotatorDeployed = 0.85;
    public final static double extensionServoSafetyPosition = 0.71;
    private double extensionServoPosition;
    private double deliveryBoxServoPosition;
    private double specimenServoPosition;
    private DcMotor Lift;
    private TouchSensor ExtensionLimit;
    private TouchSensor LiftLimit;
    private TouchSensor LiftLimit2;
    private Servo ExtensionServo;
    private Servo DeliveryBoxServo;
    private Servo GripperRotatorServo;

    private CRServo IntakeBoxServo;
    private DcMotor ExtensionSlider;
    public Servo SpecimenGripperServo;
    private LinearOpMode myOpMode;
    private int extensionSliderPosition;
    public double liftVelocity = 0.85;



    public Megalodog(LinearOpMode currentOpMode)
    {
        super(currentOpMode);
        myOpMode = currentOpMode;
        HardwareMap hardwareMap = myOpMode.hardwareMap;
    }

    public void InitializeDevices()
    {
        ExtensionServo = myOpMode.hardwareMap.get(Servo.class, "Extension");
        DeliveryBoxServo = myOpMode.hardwareMap.get(Servo.class, "DeliveryBox");
        IntakeBoxServo = myOpMode.hardwareMap.get(CRServo.class, "IntakeBox");
        SpecimenGripperServo = myOpMode.hardwareMap.get(Servo.class,"SpecimenGripper");
        GripperRotatorServo = myOpMode.hardwareMap.get(Servo.class,"GripperRotator");
        ExtensionSlider = myOpMode.hardwareMap.get(DcMotor.class, "IntakeExtension");
        Lift = myOpMode.hardwareMap.get(DcMotor.class, "Lift");
        ExtensionLimit = myOpMode.hardwareMap.get(TouchSensor.class, "ExtensionLimit");
        LiftLimit = myOpMode.hardwareMap.get(TouchSensor.class, "LiftLimit");
        LiftLimit2 = myOpMode.hardwareMap.get(TouchSensor.class, "LiftLimit2");


        SpecimenGripperServo.setDirection(Servo.Direction.REVERSE);
        GripperRotatorServo.setDirection(Servo.Direction.REVERSE);
        resetExtensionSlider(3000);

        resetLift();
     //   ((DcMotorEx)Lift).setVelocity(liftVelocity);
    }
    public void InitializePositions(boolean openExtensionBox, boolean deploySpecimenGripper)
    {
        if(openExtensionBox) {
            ExtensionServo.setPosition(extensionServoSafetyPosition);
            myOpMode.sleep(100);
        }
        deliveryBoxServoPosition = Megalodog.deliveryServoHome;
        DeliveryBoxServo.setPosition(Megalodog.deliveryServoHome);
        SpecimenGripperServo.setPosition(Megalodog.specimenServoStarting);
        if(deploySpecimenGripper) {
            GripperRotatorServo.setPosition(Megalodog.gripperRotatorDeployed);
        }
    }


    public void ReverseIntake () {
//It turns the continues servo reverse
        IntakeBoxServo.setPower(-continuousIntakePower);
        myOpMode.sleep(500);
        IntakeBoxServo.setPower(0);
    }
    public void GrabSpecimen (int waittime) {
//The servo rotates forward
        SpecimenGripperServo.setPosition(specimenServoClosed);
        myOpMode.sleep(waittime);
    }
    public void HookAndLetGo (int height, int wait){
        //turn servo and raise lift
        Lift.setTargetPosition(height);
        myOpMode.sleep(wait);
        LetGoOfSpecimen(250);
        myOpMode.sleep(wait);
    }

    public void MoveSlideAndScoop (int distanceMM,int wait) {
        if (extensionSliderPosition < extensionSliderMax) {
            ExtensionServo.setPosition(0.6);
            extensionSliderPosition += 100;
            ExtensionSlider.setTargetPosition(extensionSliderPosition);
            myOpMode.sleep(1000);
            extensionServoPosition = extensionServoFloor;
            ExtensionServo.setPosition(extensionServoPosition);
        }
    }

    public void ExchangeServoRaise(int wait) {
        ExtensionServo.setPosition(extensionServoDump);
        myOpMode.sleep(wait);
    }

    public void ExchangeSample(int wait)
    {
        IntakeBoxServo.setPower(-.14);
        myOpMode.sleep(900);
        IntakeBoxServo.setPower(0);
        ExtensionServo.setPosition(extensionServoSafetyPosition);

        myOpMode.sleep(wait);
    }

    public void ScootAndScoop(int wait)
    {
        ExtensionServo.setPosition(extensionServoFloor);
        myOpMode.sleep(200);
        IntakeBoxServo.setPower(.08);
        TurnIntakeOn(.4);
        MoveStraight(230,.15,300);

        myOpMode.sleep(800);
        TurnIntakeOff();
        myOpMode.sleep(wait);
    }

    public void RaiseLift (int heightMM, int wait){

        Lift.setTargetPosition(heightMM);
        myOpMode.sleep(wait);

    };

    public void DumpDeliveryBox (int wait){
        DeliveryBoxServo.setPosition(deliveryServoDump);
        myOpMode.sleep(wait);
    };
    public void LetGoOfSpecimen(int waittime){
        SpecimenGripperServo.setPosition(specimenServoOpen);
        myOpMode.sleep(waittime);
    };

    public void CheckSampleColor (){};

    public void TurnIntakeOn () {
        IntakeBoxServo.setPower(continuousIntakePower);
    }
    public void TurnIntakeOn(double power)
    {
        IntakeBoxServo.setPower(power);
    }
    public void TurnIntakeOff () {
        IntakeBoxServo.setPower(0);
    }

    public void Returnlift(int wait){
        RaiseLift(liftHome, wait);
    }
    public void GrabandLift(int height, int wait){
        //turn servo and raise lift
    }

    public void LiftMoveToHome(int wait){

        deliveryBoxServoPosition = deliveryServoHome;
        DeliveryBoxServo.setPosition(deliveryBoxServoPosition);
        myOpMode.sleep(100);
        Returnlift(wait);

    }

    private void resetLift()
    {

        Lift.setDirection(DcMotor.Direction.FORWARD);
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        long startTime = System.currentTimeMillis(); // Record the start time
        long maxDuration = 3000; // Maximum duration in milliseconds (3 seconds)
        while(!LiftLimit.isPressed() && !LiftLimit2.isPressed()) {
            Lift.setPower(-0.5);
            if (System.currentTimeMillis() - startTime > maxDuration) { break;}
        }
        Lift.setPower(0);

        Lift.setDirection(DcMotor.Direction.FORWARD);
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setTargetPosition(0);
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift.setPower(0.85);
    }

    private void resetExtensionSlider(long safetyDuration)
    {

        ExtensionSlider.setDirection(DcMotor.Direction.FORWARD);
        ExtensionSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ExtensionSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        long startTime = System.currentTimeMillis(); // Record the start time
        long maxDuration = safetyDuration; // Maximum duration in milliseconds (3 seconds)

        while(!ExtensionLimit.isPressed()) {
            ExtensionSlider.setPower(-0.5);
            if (System.currentTimeMillis() - startTime > maxDuration) { break;}
        }
        ExtensionSlider.setPower(0);

        ExtensionSlider.setDirection(DcMotor.Direction.FORWARD);
        ExtensionSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ExtensionSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ExtensionSlider.setTargetPosition(0);
        ExtensionSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ExtensionSlider.setPower(0.4);
        extensionSliderPosition = 0;
    }
}