package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "motorTesting", group = "Linear OpMode")
public class MotorTesting extends LinearOpMode {

    private double ticksPerDegree = 1.16666666667;

    public DcMotorEx flick;

    private final ElapsedTime flickTimer = new ElapsedTime();
    private boolean flickTimerRunning = false;


    public static double flickPower;

    public static int flickTargetPosition = 0;


    public static double P = 0.015;
    public static double I = 0.0001;
    public static double D = 0.00;
    public static double F = 0.00006;

    private static PIDFController flickController = new PIDFController(P,I,D,F);

    private Timing.Timer timer = new Timing.Timer(10);

    double previousPower = 0;  // put this as a class variable

    @Override
    public void runOpMode(){

        flick = hardwareMap.get(DcMotorEx.class,"flick");


        flick.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flick.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flick.setDirection(DcMotorSimple.Direction.FORWARD);

//        flick.setTargetPosition(0);
//
//        flick.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        waitForStart();
        while(opModeIsActive()){


            flickController.setPIDF(P,I,D,F);

            double output = flickController.calculate(
                    flick.getCurrentPosition(), flickTargetPosition
            );

            flick.setPower(output);


//             detect first commanded movement (from near-zero to non-zero)
            if (!flickTimerRunning && Math.abs(previousPower) > 0){
                flickTimer.reset();            // start counting from 0
                flickTimerRunning = true;
            }


// update previous power
            previousPower = output;


            if (flick.getCurrentPosition() - flickTargetPosition > 3){
                flickTimer.milliseconds();
            }

            if(flick.getCurrentPosition() - flickTargetPosition < 3){
                flickTimer.milliseconds();
                flickTimer.reset();
                flickTimer.log("howLong");
            }


//            if (!returned && Math.abs(flick.getCurrentPosition() - flickTargetPosition) < 5) {
//                if (timer.getMilliseconds() > 300) {     // 300ms wait
//                    flickTargetPosition = 0;             // return
//                    returned = true;
//                }
//            }
            int flickCurrentPosition = flick.getCurrentPosition();


//            if (Math.abs(flickTargetPosition - flickCurrentPosition) < 7){
//                flick.setPower(0);
//            } else {
//                flick.setPower(flickPower);
//            }
//
//                flick.setTargetPosition(flickTargetPosition);




            telemetry.addData("flickPosition", "%d", flick.getCurrentPosition());
            telemetry.addData("time", "%f", flickTimer.milliseconds());
            telemetry.update();

            FtcDashboard dashboard = FtcDashboard.getInstance();
            Telemetry dashboardTelemetry = dashboard.getTelemetry();

            dashboardTelemetry.addData("flickPosition", flick.getCurrentPosition());
            dashboardTelemetry.addData("flickTarget", flickTargetPosition);



        }
    }
}
