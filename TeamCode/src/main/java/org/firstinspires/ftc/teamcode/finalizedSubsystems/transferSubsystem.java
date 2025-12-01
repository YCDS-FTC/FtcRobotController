package org.firstinspires.ftc.teamcode.finalizedSubsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class transferSubsystem extends SubsystemBase {


    private final DcMotorEx intake2;

    public transferSubsystem(final HardwareMap hMap, final String name) {
        intake2 = hMap.get(DcMotorEx.class, "intake2");
        intake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getTransferPower(){
        double transferPower = intake2.getPower();
        return transferPower;
    }


    public void transferOff(){

        intake2.setPower(0);
    }

    public void transferOn(){
        intake2.setPower(-0.7);
    }

    public void transferLow(){
        intake2.setPower(-0.3);
    }

}
