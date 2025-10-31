package org.firstinspires.ftc.teamcode.Testing;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.HackinHoundsHardware;

import java.util.HashMap;

@Configurable
@TeleOp(name="servoTesting", group = "Linear OpMode")
public class Servotesting extends LinearOpMode {
    private HackinHoundsHardware robot = new HackinHoundsHardware();

    private static double Servopos = 0;

    HashMap<Double, Double> DINGDONG = new HashMap<Double,Double>();


    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
        DINGDONG.put(1.00, 145.2);


        while (opModeIsActive()){
        }
    }


}
