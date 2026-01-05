package org.firstinspires.ftc.teamcode.Testing;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.HackinHoundsHardware;

import java.util.HashMap;

@Configurable
@TeleOp(name="colorIndexing", group = "Linear OpMode")
public class ColorIndexing extends LinearOpMode {
    private HackinHoundsHardware robot = new HackinHoundsHardware();

    private static double Servopos = 0;

    public RevColorSensorV3 color1, color2;
    public Servo light1, light2;

    @Override
    public void runOpMode(){
        color1 = hardwareMap.get(RevColorSensorV3.class, "color1");
        color2 = hardwareMap.get(RevColorSensorV3.class, "color2");
        light1 = hardwareMap.get(Servo.class, "light1");
        light2 = hardwareMap.get(Servo.class, "light2");
        waitForStart();
        while (opModeIsActive()){


            light1.setPosition(mapColor(color1.red(), color1.green(), color1.blue()));


            light2.setPosition(mapColor(color2.getNormalizedColors().red, color2.getNormalizedColors().green, color2.getNormalizedColors().blue));




            telemetry.addData("", "%d", color1.red());
            telemetry.addData("", "%d", color1.green());
            telemetry.addData("", "%d", color1.blue());
            telemetry.addData("", "%f", color2.getNormalizedColors().red);
            telemetry.addData("", "%f", color2.getNormalizedColors().green);
            telemetry.addData("", "%f", color2.getNormalizedColors().blue);
            telemetry.addData("", color1.getNormalizedColors().toColor());
            telemetry.update();
        }
    }

    double mapColor(double r, double g, double b) {
        double max = Math.max(r, Math.max(g, b));
        if (max > 0) {
            r /= max;
            g /= max;
            b /= max;
        }

        // Determine dominant channel
        boolean blueMax = b >= g && b >= r;
        boolean greenMax = g >= b && g >= r;

        // PURPLE: Blue is highest
        if (blueMax) {
            return 0.722; // violet
        }

        // GREEN: Green is highest
        if (greenMax) {
            return 0.500; // green
        }

        // Neither
        return 0.000;
    }

}
