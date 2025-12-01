package org.firstinspires.ftc.teamcode.finalizedSubsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class stopperSubsystem extends SubsystemBase {


    private final Servo stopper;

    public stopperSubsystem(final HardwareMap hMap, final String name) {
        stopper = hMap.get(Servo.class, "stopper");

    }


    public double getStopperPosition(){
        double stopperPos = stopper.getPosition();
        return stopperPos;
    }

    public void stopperOff(){
        stopper.setPosition(.47);
    }

    public void stopperOn(){
        stopper.setPosition(.67);
    }


}
