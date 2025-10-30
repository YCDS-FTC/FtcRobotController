package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "motorTesting", group = "Linear OpMode")
public class MotorTesting extends LinearOpMode {

    private double ticksPerDegree = 1.16666666667;

    public DcMotorEx flick;


    public static double flickPower;

    public static int flickTargetPosition;

    public static int flickCurrentPosition;

    @Override
    public void runOpMode(){

        flick = hardwareMap.get(DcMotorEx.class,"flick");

        flick.setTargetPosition(0);

        flick.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flick.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        flickCurrentPosition = flick.getCurrentPosition();

        waitForStart();
        while(opModeIsActive()){





            if (Math.abs(flickTargetPosition - flickCurrentPosition) < 7){
                flick.setPower(0);
            } else {
                flick.setPower(flickPower);
            }
            flick.setTargetPosition(flickTargetPosition);



            telemetry.addData("flickPosition", "%d", flick.getCurrentPosition());
            telemetry.update();

        }
    }
}
