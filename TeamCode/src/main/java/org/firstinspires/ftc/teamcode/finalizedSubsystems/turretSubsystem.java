package org.firstinspires.ftc.teamcode.finalizedSubsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

public class turretSubsystem extends SubsystemBase {


    private final DcMotorEx turret;


    private double turretPower = 0;

    public turretSubsystem(final HardwareMap hMap, final String name) {
        turret = hMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getTurretPosition(){
        double turretAngle = turret.getCurrentPosition()/4.233;
        return turretAngle;
    }

    public void setTurretPower(double output){

         turretPower = output;
    }




    @Override
    public void periodic(){
        turret.setPower(turretPower);
    }

}