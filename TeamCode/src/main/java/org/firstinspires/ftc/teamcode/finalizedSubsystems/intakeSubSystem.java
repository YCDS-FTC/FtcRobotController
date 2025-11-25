package org.firstinspires.ftc.teamcode.finalizedSubsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class intakeSubSystem extends SubsystemBase {


    private final DcMotorEx intake;

    public intakeSubSystem(final HardwareMap hMap, final String name) {
        intake = hMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void intakeOff(){
        intake.setPower(0);
    }

    public void intakeOn(){
        intake.setPower(0.6);
    }

    public void intakeLow(){
        intake.setPower(0.1);
    }

}
