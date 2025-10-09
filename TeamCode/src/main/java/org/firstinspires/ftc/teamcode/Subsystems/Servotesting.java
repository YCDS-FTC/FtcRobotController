package org.firstinspires.ftc.teamcode.Subsystems;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.HackinHoundsHardware;

@Configurable
@TeleOp(name="servoTesting", group = "Linear OpMode")
public class Servotesting extends LinearOpMode {
    private HackinHoundsHardware robot = new HackinHoundsHardware();

    private static double Servopos = 0;


    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();


        while (opModeIsActive()){
            robot.test.setPosition(Servopos);
        }
    }


}
