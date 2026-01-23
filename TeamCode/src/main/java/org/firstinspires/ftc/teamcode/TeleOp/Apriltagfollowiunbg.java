package org.firstinspires.ftc.teamcode.TeleOp;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Hardware.HackinHoundsHardware;

@Configurable
//@TeleOp(name="apriltagfollowing", group="Linear OpMode")
public class Apriltagfollowiunbg extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private HackinHoundsHardware robot = new HackinHoundsHardware();


    private static double Kp = 3;
    private static double Ki = 0;
    private static double Kd = 0.05;

    double integralSum = 0;
    double lastError = 0;
    ElapsedTime timer = new ElapsedTime();


    boolean targetVisible = false;

    private static double tolerance = 2.0;
    private static double desiredOffset = 0;

    private static double horizontalOffset;

    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        telemetry.addData("Status", "Initialized");
        telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.update();


        waitForStart();
        runtime.reset();


        limelight.start();
        runtime.reset();


        while(opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result.isValid()) {
                // Read tx (in degrees)
                double tx = result.getTx();

// Get the robot's current heading from IMU (in radians)
                double currentAngle = robot.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

// Compute the desired *absolute* angle in IMU space
                double targetAngle = currentAngle + Math.toRadians(tx);

// PID error becomes how far the robot is from that new target angle
                double turnPower = PIDControl(targetAngle, currentAngle);
//
//                double targetAngle = Math.toRadians(-result.getTx());
//                double currentAngle = robot.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
//                double turnPower = PIDControl(targetAngle, currentAngle);

                robot.leftFront.setPower(turnPower);
                robot.leftBack.setPower(turnPower);
                robot.rightBack.setPower(-turnPower);
                robot.rightFront.setPower(-turnPower);


                telemetry.addData("Limelight tx", result.getTx());
                telemetry.addData("Target Angle (deg)", Math.toDegrees(targetAngle));
                telemetry.addData("IMU Angle (deg)", Math.toDegrees(currentAngle));
                telemetry.addData("Turn Power", turnPower);
            } else {
                telemetry.addLine("No tag detected");
                robot.rightFront.setPower(0);
                robot.rightBack.setPower(0);
                robot.leftBack.setPower(0);
                robot.leftFront.setPower(0);
            }

            telemetry.update();

        }
    }
    double PIDControl( double reference, double state){
        double error = angleWrap(reference - state);

        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        timer.reset();

        double output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
        return output;
    }

    double angleWrap ( double radians){
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }
}
