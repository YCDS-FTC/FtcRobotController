package org.firstinspires.ftc.teamcode.Testing;

import android.content.OperationApplicationException;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.util.InterpLUT;

import java.util.HashMap;

@TeleOp(name="interpolation", group="Linear OpMode")
public class interpaolation extends OpMode {
    InterpLUT shooterPower = new InterpLUT();


    @Override
    public void init(){
        shooterPower.add(5,1060);
        shooterPower.add(10,1100);

        shooterPower.createLUT();
    }

    @Override
    public void loop(){
        double power = shooterPower.get(7.5);
        telemetry.addData("power", power);
        telemetry.update();
    }




}
