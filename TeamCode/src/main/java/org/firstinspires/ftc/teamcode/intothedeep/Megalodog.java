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
    public final static int extensionSliderMax = 900;
    public final static int liftLowerBasket = 1300;
    public final static int liftHome = 30;
    public final static int liftUpperBasket = 3200;
    public final static int liftLowerSpecimenBar = 500;
    public final static int liftUpperSpecimenBar = 2170;
    public final static int liftPullSpecimenFromUpperBar = 1120;  // 1060 less than upper spec bar
    public final static int liftSnapSpecimen = 200;
    public final static int liftGetSpecimenFromWall = 500;
    public final static int liftHangOnLowerBar = 2000;
    public final static int liftHangOnUpperBar = 1000;
    // .89 was good when angled higher
    //  .93 was config servo at lower angle
    public static double extensionServoFloor = 0.69;
    public final static double extensionServoDump = 0.23;  // was                .235
    public final static double extensionServoSafetyPosition = 0.4;
    public final static double deliveryServoHome = 0.04;
    public final static double deliveryServoDump = 0.55;
    public final static double deliveryServoSpeciem = 0.9;
    public final static double specimenServoOpen = 0.73;
    public final static double specimenServoClosed = 0.62;
    public final static double specimenServoStarting = 0.527;
    public final static double continuousIntakePower = 0.4;
    public final static double continuousIntakeDumpPower = 0.6;
    public final static double gripperRotatorStarting = 0.44;
    public final static double gripperRotatorDeployed = 0.742;
    public final static double gripperRotatorDowntoGrab = 0.75;
    public final static double extensionBoxRotatorStarting = 0.544;
    public final static double extensionBoxRotatorDumping = 0.695;

    private double extensionServoPosition;
    private double deliveryBoxServoPosition;
    private double specimenServoPosition;
    private DcMotor Lift;
    private TouchSensor ExtensionLimit;
    private TouchSensor LiftLimit;
    private TouchSensor LiftLimit2;
    private TouchSensor WallFinder;
    public Servo ExtensionServo;
    public Servo DeliveryBoxServo;
    public Servo GripperRotatorServo;
    public Servo ExtensionBoxRotation;

    public CRServo IntakeBoxServo;
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
        WallFinder = myOpMode.hardwareMap.get(TouchSensor.class, "WallFinder");
        ExtensionBoxRotation = myOpMode.hardwareMap.get(Servo.class, "ExtensionBoxRotator");

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
        ExtensionBoxRotation.setPosition(Megalodog.extensionBoxRotatorStarting);
        GripperRotatorServo.setPosition(Megalodog.gripperRotatorStarting);
        if(deploySpecimenGripper) {
            GripperRotatorServo.setPosition(Megalodog.gripperRotatorDeployed);
        }
    }

    public boolean MoveUntilButton(double velocity, int maxDistance)
    {
        int distanceTravelled = 0;
        int moveDistance = 15;
        if(maxDistance<0)
        {
            moveDistance *= -1;
        }
        while(!WallFinder.isPressed() && distanceTravelled < Math.abs(maxDistance))
        {
            MoveStraight(moveDistance, velocity, 0);
            distanceTravelled += 15;
            myOpMode.sleep(200);
        }
        return (distanceTravelled < Math.abs(maxDistance));
    }

    public void ReverseIntake () {
//It turns the continues servo reverse
        IntakeBoxServo.setPower(-continuousIntakePower);
        myOpMode.sleep(500);
        IntakeBoxServo.setPower(0);
    }
    public void GrabSpecimenAndLift (int waittime) {

        SpecimenGripperServo.setPosition(specimenServoClosed);
        myOpMode.sleep(500);
        Lift.setTargetPosition(liftHome+700);
        myOpMode.sleep(waittime);
    }

    public void DeployExtensionServo(int wait)
    {
        ExtensionServo.setPosition(extensionServoSafetyPosition);
        myOpMode.sleep(wait);
    }
    public void HookAndLetGo (int height, int waitBetweenDropAndLetGo, int waitAfter){
        //turn servo and raise lift
        Lift.setPower(0.5);
        Lift.setTargetPosition(height);
        //myOpMode.sleep(waitBetweenDropAndLetGo);
        while (Lift.getCurrentPosition() > height+30) { // wait
        }
        LetGoOfSpecimen(0);
        Lift.setPower(0.85);
        myOpMode.sleep(waitAfter);
    }

    public void PutGripperAway()
    {
        SpecimenGripperServo.setPosition(specimenServoClosed);
        myOpMode.sleep(200);
        GripperRotatorServo.setPosition(gripperRotatorStarting);
    }

    public void DeployAndOpenSpecimenGripper(int wait)
    {
        GripperRotatorServo.setPosition(gripperRotatorDeployed);
        myOpMode.sleep(100);
        SpecimenGripperServo.setPosition(specimenServoOpen);
        myOpMode.sleep(wait);

    }
    public void OpenGripperForWall(int wait)
    {
        GripperRotatorServo.setPosition(gripperRotatorDowntoGrab);
        myOpMode.sleep(100);
        SpecimenGripperServo.setPosition(specimenServoOpen-0.15);
        myOpMode.sleep(wait);

    }
    public void LevelGripper(int wait)
    {
        GripperRotatorServo.setPosition(gripperRotatorDeployed);
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
        IntakeBoxServo.setPower(-.2);  // was -.18
        myOpMode.sleep(1000); // was 1000
        IntakeBoxServo.setPower(.2);
        myOpMode.sleep(1000);
        IntakeBoxServo.setPower(0);
        ExtensionServo.setPosition(extensionServoSafetyPosition);

        myOpMode.sleep(wait);
    }

    public void ScootAndScoop(int wait)
    {
        ScootAndScoop(wait, 200);
    }

    public void ScootAndScoop(int wait, int scootDistance)
    {
        ExtensionServo.setPosition(extensionServoFloor);
        TurnIntakeOn(.22);
        //myOpMode.sleep(100); // was 200
        MoveStraight(scootDistance,.2,100);
        ExtensionServo.setPosition(extensionServoFloor-0.018);
        myOpMode.sleep(1000);
        ExtensionServo.setPosition(extensionServoFloor);
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

    //    Lift.setDirection(DcMotor.Direction.FORWARD);
    //    Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setTargetPosition(0);
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double lift_max_velocity = (312 / 60) * 537.7;
        ((DcMotorEx)Lift).setVelocity(.85*lift_max_velocity);
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

     //   ExtensionSlider.setDirection(DcMotor.Direction.FORWARD);
      //  ExtensionSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ExtensionSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ExtensionSlider.setTargetPosition(0);
        ExtensionSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ExtensionSlider.setPower(0.4);
        extensionSliderPosition = 0;
    }
}