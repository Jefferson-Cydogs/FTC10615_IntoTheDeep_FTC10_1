package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@Disabled
@TeleOp(name = "SampleColorDetection (Blocks to Java)")
public class ColorSensorTest extends LinearOpMode {

    private DigitalChannel RedSampleLED;
    private DigitalChannel GreenSampleLED;
    private ColorSensor SampleVisor_REV_ColorRangeSensor;

    /**
     * Describe this function...
     */
    @Override
    public void runOpMode() {
        int gain;
        boolean SampleInsideIntake;
        NormalizedRGBA normalizedColors;
        int color;
        float hue;



        RedSampleLED = hardwareMap.get(DigitalChannel.class, "RedSampleLED");
        GreenSampleLED = hardwareMap.get(DigitalChannel.class, "GreenSampleLED");
        SampleVisor_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "SampleVisor");

        // Initialize devices
        RedSampleLED.setMode(DigitalChannel.Mode.OUTPUT);
        GreenSampleLED.setMode(DigitalChannel.Mode.OUTPUT);
        RedSampleLED.setState(true);
        GreenSampleLED.setState(true);
        // Initialize variables
        gain = 2;
        SampleInsideIntake = false;
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                ((NormalizedColorSensor) SampleVisor_REV_ColorRangeSensor).setGain(gain);
                // Read color from the sensor.
                normalizedColors = ((NormalizedColorSensor) SampleVisor_REV_ColorRangeSensor).getNormalizedColors();
                // Convert RGB values to Hue, Saturation, and Value.
                color = normalizedColors.toColor();
                hue = JavaUtil.colorToHue(color);
                // Use hue to determine if it's red, green, blue, etc..
                if (((OpticalDistanceSensor) SampleVisor_REV_ColorRangeSensor).getLightDetected() > 0.1) {
                    telemetry.addData("Sample inside Intake", "GOT IT!!");
                    if (!SampleInsideIntake) {
                        if (hue < 60) {
                            telemetry.addData("Color", "Red");
                            RedSampleLED.setState(false);
                            GreenSampleLED.setState(true);
                        } else if (hue <= 90) {
                            telemetry.addData("Color", "Yellow");
                            RedSampleLED.setState(true);
                            GreenSampleLED.setState(false);
                        } else if (hue >= 210 && hue <= 250) {
                            telemetry.addData("Color", "Blue");
                            RedSampleLED.setState(false);
                            GreenSampleLED.setState(true);
                        }
                        SampleInsideIntake = true;
                    }
                } else {
                    telemetry.addData("Sample inside Intake", "Not yet");
                    if (SampleInsideIntake) {
                        RedSampleLED.setState(true);
                        GreenSampleLED.setState(true);
                        SampleInsideIntake = false;
                    }
                }
                telemetry.update();
            }
        }
    }
}