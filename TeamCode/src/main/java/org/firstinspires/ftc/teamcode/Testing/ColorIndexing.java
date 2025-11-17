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

            if (color2.green() > 150) {
                light2.setPosition(0.5);
            }

            telemetry.addData("", "%d", color1.red());
            telemetry.addData("", "%d", color1.green());
            telemetry.addData("", "%d", color1.blue());
            telemetry.addData("", "%d", color2.red());
            telemetry.addData("", "%d", color2.green());
            telemetry.addData("", "%d", color2.blue());
            telemetry.update();
        }
    }


}
