package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.HackinHoundsHardware;


@Configurable
public  class shooterTesting extends OpMode {
    private HackinHoundsHardware robot = new HackinHoundsHardware();

    public static double shooterPower = 0;
    @Override
    public void init(){

    }

    @Override
    public void loop(){
//        return 0;
    }


}
