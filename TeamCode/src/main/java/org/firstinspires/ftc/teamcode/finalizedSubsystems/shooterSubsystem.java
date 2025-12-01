package org.firstinspires.ftc.teamcode.finalizedSubsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class shooterSubsystem extends SubsystemBase {


    private final DcMotorEx shooter;

    private double shooterPower = 0;

    private double turretPower = 0;

    public shooterSubsystem(final HardwareMap hMap, final String name) {
        shooter = hMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }



    public double getShooterVelocity(){
        double shooterVelocity = shooter.getVelocity();

        return shooterVelocity;

    }

    public void setShooterPower(double output){

         shooterPower = output;
    }

    public void shooterOff(){
        shooter.setPower(0);
    }





    @Override
    public void periodic(){
        shooter.setPower(shooterPower);
    }

}