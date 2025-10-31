package org.firstinspires.ftc.teamcode.Testing;

import android.content.OperationApplicationException;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;

@TeleOp(name="Mechanum", group="Linear OpMode")
public class interpaolation extends OpMode {
    HashMap<Double, Double> DINGDONG = new HashMap<Double,Double>();

    @Override
    public void init(){

    }

    @Override
    public void loop(){
        DINGDONG.put(1.00, 145.2);

    }


}
