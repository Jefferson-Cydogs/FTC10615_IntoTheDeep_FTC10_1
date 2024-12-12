package org.firstinspires.ftc.teamcode.configs;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.EventTracker;
import org.firstinspires.ftc.teamcode.intothedeep.Megalodog;


@TeleOp
public class Config_ExtensionRotator extends LinearOpMode {

    private DcMotor BackLeftWheel;
    private DcMotor FrontLeftWheel;
    private DcMotor BackRightWheel;
    private DcMotor FrontRightWheel;
    private DcMotor ExtensionSlider;
    private int extensionSliderPosition;
    private DcMotor Lift;
    private TouchSensor ExtensionLimit;
    private TouchSensor LiftLimit;
    private TouchSensor LiftLimit2;
    private TouchSensor WallFinder;
    private Servo ExtensionServo;
    private Servo DeliveryBoxServo;
    private Servo ExtensionBoxRotation;
    private Servo GripperRotatorServo;
    private CRServo IntakeBoxServo;
    public Servo SpecimenGripperServo;
    private double extensionServoPosition;
    private DigitalChannel RedLEDLeft;
    private DigitalChannel RedLEDRight;
    private DigitalChannel GreenLEDLeft;
    private DigitalChannel GreenLEDRight;

    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;
    private double deliveryBoxServoPosition;
    private double specimenServoPosition;
    private float gamepad1_RightStickYValue;
    private float gamepad1_RightStickXValue;
    private float gamepad1_LeftStickYValue;
    private float gamepad1_LeftStickXValue;
    private float gamepad1_TriggersValue;
    private double Straight;
    private double Strafe;
    private double Rotate;
    private double FastStraight;
    private double FastStrafe;
    private double highSpeedDrive = 0.7;
    private double lowSpeedDrive = 0.3;
    private double rotateSpeedDrive = 0.55;
    private int liftGoToPosition = 0;
    private double currentIntakePower = Megalodog.continuousIntakePower;
    private boolean triggerSpecimenGripperOpen = false;
    private boolean allowDriving = true;

    private double currentExtensionBoxRotationPosition = Megalodog.extensionBoxRotatorStarting;
    private double extensionBoxRotationSpeed = 0.035;
    private double currentExtensionBoxPosition = Megalodog.extensionServoSafetyPosition;
    private double extensionBoxSpeed = 0.035;
    double extensionBoxJoystick;

    private ElapsedTime currentTimer;
    private EventTracker eventTracker;
    private int red;
    private int green;
    private int blue;
    private double distance;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    // extensionIntake
    //    floor  .81
    //    above submerssible bar   .7
    //    dump .2
    //    starting position

    @Override
    public void runOpMode() {

        initializeWheels();
        initializeDevices();


        currentTimer = new ElapsedTime();


        eventTracker = new EventTracker();

        waitForStart();
        if (opModeIsActive()) {
            // RUN BLOCKS:
            initializePositions();
            while (opModeIsActive()) {

                // LOOP BLOCKS:

                manageDriverControls();
                manageManipulatorControls();

                // get color settings


                if(eventTracker.doEvent("Telemetry",currentTimer.seconds(),0.3))
                {

                    telemetry.update();
                }
            }
        }
    }

    private void manageDriverControls()
    {
        if(gamepad1.square)
        {
            resetExtensionSlider(1000);
        }
        if(gamepad1.circle)
        {
            resetLift();
        }
        if(gamepad1.dpad_up)
        {
            allowDriving=true;
        }
        if(gamepad1.triangle) //  hook specimen
        {
            Lift.setTargetPosition(Megalodog.liftPullSpecimenFromUpperBar);
            //schedule an open of specimen gripper
            triggerSpecimenGripperOpen = true;
        }
        if(gamepad1.cross)
        {
            if(GripperRotatorServo.getPosition() > Megalodog.gripperRotatorDeployed - 0.1) {
                SpecimenGripperServo.setPosition(Megalodog.specimenServoClosed);
                GripperRotatorServo.setPosition(Megalodog.gripperRotatorStarting);
            }
            else {
                GripperRotatorServo.setPosition(Megalodog.gripperRotatorDeployed);
            }
        }
        if(gamepad1.dpad_up)
        {
            if(eventTracker.doEvent("Extension Box",currentTimer.seconds(),0.05)) {
                currentExtensionBoxPosition += 0.005;
                currentExtensionBoxPosition = Math.max(0.0, Math.min(currentExtensionBoxPosition, 1.0));
                ExtensionServo.setPosition(currentExtensionBoxPosition);
                telemetry.addData("extension arm:", currentExtensionBoxPosition);
            }
        }
        if(gamepad1.dpad_down)
        {
            if(eventTracker.doEvent("Extension Box",currentTimer.seconds(),0.05)) {
                currentExtensionBoxPosition -= 0.005;
                currentExtensionBoxPosition = Math.max(0.0, Math.min(currentExtensionBoxPosition, 1.0));
                ExtensionServo.setPosition(currentExtensionBoxPosition);
                telemetry.addData("extension arm:", currentExtensionBoxPosition);
            }
        }
        if(gamepad1.dpad_left)
        {
            if(eventTracker.doEvent("Extension Box Rotator",currentTimer.seconds(),0.05)) {
                currentExtensionBoxRotationPosition += 0.005;
                currentExtensionBoxRotationPosition = Math.max(0.0, Math.min(currentExtensionBoxRotationPosition, 1.0));
                ExtensionBoxRotation.setPosition(currentExtensionBoxRotationPosition);
                telemetry.addData("extension rotate:", currentExtensionBoxRotationPosition);
            }
        }
        if(gamepad1.dpad_right)
        {
            if(eventTracker.doEvent("Extension Box Rotator",currentTimer.seconds(),0.05)) {
                currentExtensionBoxRotationPosition -= 0.005;
                currentExtensionBoxRotationPosition = Math.max(0.0, Math.min(currentExtensionBoxRotationPosition, 1.0));
                ExtensionBoxRotation.setPosition(currentExtensionBoxRotationPosition);
                telemetry.addData("extension rotate:", currentExtensionBoxRotationPosition);
            }
        }

    }
    private void manageManipulatorControls()
    {

        if(-gamepad2.right_stick_y > 0.2)
        {
            if(eventTracker.doEvent("Extension Box Rotator",currentTimer.seconds(),0.05)) {
                currentExtensionBoxRotationPosition += extensionBoxRotationSpeed;
                currentExtensionBoxRotationPosition = Math.max(0.0, Math.min(currentExtensionBoxRotationPosition, 1.0));
                ExtensionBoxRotation.setPosition(currentExtensionBoxRotationPosition);
                telemetry.addData("extension rotate:", currentExtensionBoxRotationPosition);
            }
        }
        if(-gamepad2.right_stick_y < -0.2)
        {
            if(eventTracker.doEvent("Extension Box Rotator",currentTimer.seconds(),0.05)) {
                currentExtensionBoxRotationPosition -= extensionBoxRotationSpeed;
                currentExtensionBoxRotationPosition = Math.max(0.0, Math.min(currentExtensionBoxRotationPosition, 1.0));
                ExtensionBoxRotation.setPosition(currentExtensionBoxRotationPosition);
                telemetry.addData("extension rotate:", currentExtensionBoxRotationPosition);
            }
        }
        if(-gamepad2.left_stick_y > 0.2)
        {
            if(eventTracker.doEvent("Extension Box",currentTimer.seconds(),0.05)) {
                currentExtensionBoxPosition += extensionBoxSpeed;
                currentExtensionBoxPosition = Math.max(0.0, Math.min(currentExtensionBoxPosition, 1.0));
                ExtensionServo.setPosition(currentExtensionBoxPosition);
                telemetry.addData("extension arm:", currentExtensionBoxPosition);
            }
        }
        if(-gamepad2.left_stick_y < -0.2)
        {
            if(eventTracker.doEvent("Extension Box",currentTimer.seconds(),0.05)) {
                currentExtensionBoxPosition -= extensionBoxSpeed;
                currentExtensionBoxPosition = Math.max(0.0, Math.min(currentExtensionBoxPosition, 1.0));
                ExtensionServo.setPosition(currentExtensionBoxPosition);
                telemetry.addData("extension arm:", currentExtensionBoxPosition);
            }
        }

    }

    private void hangRobot()
    {
        Lift.setTargetPosition(Megalodog.liftUpperSpecimenBar-300);
        ExtensionServo.setPosition(.5);
        FrontRightWheel.setPower(0);
        BackRightWheel.setPower(0);
        FrontLeftWheel.setPower(0);
        BackLeftWheel.setPower(0);

        // Set the direction of the wheels.  Because of how the wheels are installed, one side
        //   has to be reverse.
        FrontRightWheel.setDirection(DcMotor.Direction.FORWARD);
        BackRightWheel.setDirection(DcMotor.Direction.FORWARD);
        FrontLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        BackLeftWheel.setDirection(DcMotor.Direction.REVERSE);

        // > Set motors' ZeroPower behavior
        FrontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // > Clear Encoders of prior data
        FrontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeftWheel.setTargetPosition(0);
        FrontRightWheel.setTargetPosition(0);
        BackLeftWheel.setTargetPosition(0);
        BackRightWheel.setTargetPosition(0);

        // > Set some motors' modes different from RUN_WITHOUT_ENCODER (default)
        FrontLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Megalodog myBot = new Megalodog(this);
        myBot.MoveStraight(100,.4, 100);
        Lift.setTargetPosition(Megalodog.liftHome);
    }
    private void initializePositions()
    {

        ExtensionServo.setPosition(Megalodog.extensionServoSafetyPosition);
        deliveryBoxServoPosition = Megalodog.deliveryServoHome;
        DeliveryBoxServo.setPosition(Megalodog.deliveryServoHome);
        SpecimenGripperServo.setPosition(Megalodog.specimenServoStarting);
        GripperRotatorServo.setPosition(Megalodog.gripperRotatorStarting);
        ExtensionBoxRotation.setPosition(Megalodog.extensionBoxRotatorStarting);
        RedLEDLeft.setState(true);
        RedLEDRight.setState(true);
        GreenLEDRight.setState(true);
        GreenLEDLeft.setState(true);

        RedLEDLeft.setState(true);
        GreenLEDLeft.setState(false);

        GreenLEDRight.setState(true);
        RedLEDRight.setState(false);

    }

    private void driveChassis()
    {
        gamepad1_RightStickYValue = -gamepad1.right_stick_y;
        gamepad1_RightStickXValue = gamepad1.right_stick_x;
        gamepad1_LeftStickYValue = -gamepad1.left_stick_y;
        gamepad1_LeftStickXValue = gamepad1.left_stick_x;
        gamepad1_TriggersValue = gamepad1.right_trigger - gamepad1.left_trigger;
        if(Math.abs(gamepad1_RightStickYValue) > 0.2)
        {
            checkExtensionBoxForDrive();
           ExtensionBoxRotation.setPosition(Megalodog.extensionBoxRotatorStarting);
        }
        if (gamepad1_RightStickYValue != 0 || gamepad1_RightStickXValue != 0 || gamepad1_LeftStickYValue != 0 || gamepad1_LeftStickXValue != 0 || gamepad1_TriggersValue != 0) {
            // Set robot's move forward(+) or backwards(-) power
            Straight = lowSpeedDrive * (0.75 * Math.pow(gamepad1_LeftStickYValue, 3) + 0.25 * gamepad1_LeftStickYValue);
            // Set robot's strafe right(+) or left(-) power
            Strafe = lowSpeedDrive * (0.75 * Math.pow(gamepad1_LeftStickXValue, 3) + 0.25 * gamepad1_LeftStickXValue);
            // Set robot's clockwise(+) or counter-clockwise(-) rotation power
            Rotate = rotateSpeedDrive * (0.75 * Math.pow(gamepad1_TriggersValue, 3) + 0.25 * gamepad1_TriggersValue);
            // Set robot's fast move forward(+) or backwards(-) power
            FastStraight = highSpeedDrive * gamepad1_RightStickYValue;
            // Set robot's fast strafe right(+) or left(-) power
            FastStrafe = highSpeedDrive * gamepad1_RightStickXValue;
            BackLeftWheel.setPower((((Straight + FastStraight) - Strafe) - FastStrafe) + Rotate);
            BackRightWheel.setPower((Straight + FastStraight + Strafe + FastStrafe) - Rotate);
            FrontLeftWheel.setPower(Straight + FastStraight + Strafe + FastStrafe + Rotate);
            FrontRightWheel.setPower((((Straight + FastStraight) - Strafe) - FastStrafe) - Rotate);
        } else {
            // Stop all motors if their controls are not touched
            BackLeftWheel.setPower(0);
            BackRightWheel.setPower(0);
            FrontLeftWheel.setPower(0);
            FrontRightWheel.setPower(0);
        }
    }

    private void initializeWheels()
    {
        BackLeftWheel = hardwareMap.get(DcMotor.class, "BackLeftWheel");
        FrontLeftWheel = hardwareMap.get(DcMotor.class, "FrontLeftWheel");
        BackRightWheel = hardwareMap.get(DcMotor.class, "BackRightWheel");
        FrontRightWheel = hardwareMap.get(DcMotor.class, "FrontRightWheel");

        // INITIALIZATION BLOCKS:
        // > Reverse motors'/servos' direction as needed
        BackLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        FrontLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        BackRightWheel.setDirection(DcMotor.Direction.FORWARD);
        FrontRightWheel.setDirection(DcMotor.Direction.FORWARD);
        // > Set motors' ZeroPower behavior
        BackLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BackRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FrontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FrontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // > Clear Encoders of prior data
        FrontLeftWheel.setPower(0);
        FrontRightWheel.setPower(0);
        BackLeftWheel.setPower(0);
        BackRightWheel.setPower(0);
        BackLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // > Set some motors' modes different from RUN_WITHOUT_ENCODER (default)
        BackLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void initializeDevices()
    {
        ExtensionServo = hardwareMap.get(Servo.class, "Extension");
        ExtensionBoxRotation = hardwareMap.get(Servo.class, "ExtensionBoxRotator");
        DeliveryBoxServo = hardwareMap.get(Servo.class, "DeliveryBox");
        IntakeBoxServo = hardwareMap.get(CRServo.class, "IntakeBox");
        SpecimenGripperServo = hardwareMap.get(Servo.class,"SpecimenGripper");
        GripperRotatorServo = hardwareMap.get(Servo.class,"GripperRotator");
        ExtensionSlider = hardwareMap.get(DcMotor.class, "IntakeExtension");
        Lift = hardwareMap.get(DcMotor.class, "Lift");
        ExtensionLimit = hardwareMap.get(TouchSensor.class, "ExtensionLimit");
        LiftLimit = hardwareMap.get(TouchSensor.class, "LiftLimit");
        LiftLimit2 = hardwareMap.get(TouchSensor.class, "LiftLimit2");
        WallFinder = hardwareMap.get(TouchSensor.class, "WallFinder");
        // LEDLeft and LEDRight

        RedLEDLeft = hardwareMap.get(DigitalChannel.class, "LeftLEDRed");
        RedLEDLeft.setMode(DigitalChannel.Mode.OUTPUT);
        RedLEDRight = hardwareMap.get(DigitalChannel.class, "RightLEDRed");
        RedLEDRight.setMode(DigitalChannel.Mode.OUTPUT);
        GreenLEDLeft = hardwareMap.get(DigitalChannel.class, "LeftLEDGreen");
        GreenLEDLeft.setMode(DigitalChannel.Mode.OUTPUT);
        GreenLEDRight = hardwareMap.get(DigitalChannel.class, "RightLEDGreen");
        GreenLEDRight.setMode(DigitalChannel.Mode.OUTPUT);

        colorSensor = hardwareMap.get(ColorSensor.class,"ColorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "ColorSensor");

        SpecimenGripperServo.setDirection(Servo.Direction.REVERSE);

        GripperRotatorServo.setDirection(Servo.Direction.REVERSE);
        resetExtensionSlider(3000);

        resetLift();
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
        ExtensionSlider.setPower(0.8);
        extensionSliderPosition = 0;
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

        // LAJIRAFA_MAX_VELOCITY = (312 / 60) * 537.7;
        // 312 is
        // 537.7 is
        //double liftMaxVelocity = (312 / 60) * 537.7;

        Lift.setPower(0.8);
    }

    private void checkExtensionServoSafety()
    {
        if(ExtensionServo.getPosition() > Megalodog.extensionServoSafetyPosition+0.04)
        {
            ExtensionServo.setPosition(Megalodog.extensionServoSafetyPosition);
        }
    }

    private boolean checkIsLiftDown()  // check if lift is down and delivery is home
    {
        return (Lift.getCurrentPosition() < 150 && DeliveryBoxServo.getPosition() > Megalodog.deliveryServoHome-0.1);
    }
    private boolean checkIsLiftUp()
    {
        return (Lift.getCurrentPosition() > Megalodog.liftLowerBasket);
    }

    private boolean checkIsIntakeUp()
    {
        return (ExtensionServo.getPosition() > Megalodog.extensionServoDump-0.2);
    }
    private boolean checkIsExtensionHome()
    {
        return (ExtensionSlider.getCurrentPosition() < 100);
    }
    private void checkExtensionBoxForDrive()
    {
        if(ExtensionServo.getPosition() < Megalodog.extensionServoSafetyPosition - 0.04)
        {
            ExtensionServo.setPosition(Megalodog.extensionServoSafetyPosition);
        }
    }

    private boolean checkRotatorGripperIsDeployed()
    {
        return (GripperRotatorServo.getPosition() > Megalodog.gripperRotatorDeployed-0.05);
    }

    private void stopWheels()
    {
        BackLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeftWheel.setPower(0);
        FrontRightWheel.setPower(0);
        BackLeftWheel.setPower(0);
        BackRightWheel.setPower(0);
    }

    private void reFloatWheels()
    {
        BackLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BackRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FrontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FrontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }


}
