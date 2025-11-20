package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.Rev9AxisImuOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Hardware.HackinHoundsHardware;

import java.lang.annotation.Native;


@Config
@TeleOp(name = "Mechanum", group = "Linear OpMode")
public class HackinHounds_Mechanum extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private HackinHoundsHardware robot = new HackinHoundsHardware();


    double shift = 1;



    public static double transferPower = 0;
    public static double intakePower = 0;

    // intake = 0.5
    // transfer = 0.8


    private static double turret_tPERd = 4.233;
    private static double angleWant = 0;
    private static double slow = 1;

    private static double p = 0.02;
    private static double i = 0.002;
    private static double d = 0.0005;
    private static double f = 0;

    PIDFController turretController = new PIDFController(p,i,d,f);

    @Override
    public void init(){
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.update();
        robot.limelight.pipelineSwitch(0);
    }

    @Override
    public void start(){
        robot.limelight.start();

    }


    @Override
    public void loop(){



        turretController.setPIDF(p,i,d,f);


        LLResult result = robot.limelight.getLatestResult();

        double tx = result.getTx();
        double ty = result.getTy();


        if (gamepad1.dpad_up) {
            shift = 1;
        } else if (gamepad1.dpad_right) {
            shift = 0.75;
        } else if (gamepad1.dpad_down) {
            shift = 0.5;
        }

        double facing = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double y = -gamepad1.left_stick_y;
        //double y = 0;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double rotX = x * Math.cos(-facing) - y * Math.sin(-facing);
        rotX = rotX * 1.1;
        double rotY = x * Math.sin(-facing) + y * Math.cos(-facing);

        double d = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(rx), 1);

        double lf = (rotY + rotX + rx) / d;
        double lb = (rotY - rotX + rx) / d;
        double rf = (rotY - rotX - rx) / d;
        double rb = (rotY + rotX - rx) / d;

        robot.leftBack.setVelocity(3000 * lb * shift);
        robot.leftFront.setVelocity(3000 * lf * shift);
        robot.rightBack.setVelocity(3000 * rb * shift);
        robot.rightFront.setVelocity(3000 * rf * shift);

        robot.intake.setPower(intakePower);
        robot.intake2.setPower(transferPower);

        if (gamepad1.back) {
            robot.imu.resetYaw();
        }



        double turretAngle = robot.turret.getCurrentPosition()/turret_tPERd;
        double target = normA( turretAngle - tx);
        if (target > 135) {target = 135;} else if (target < -135) {target = -135;}
        double error = target - turretAngle;
        double turretPower = clamp(error * slow, -1, 1);
//            turret.setPower(turretController.calculate(turretAngle, target));
        double output = (turretController.calculate(turretAngle, target));

        robot.turret.setPower(output);

        telemetry.addData("turretPos", "%d", robot.turret.getCurrentPosition());
        telemetry.addData("turretAngle", "%f", turretAngle);
        telemetry.addData("turretTarget", "%f", target);
        telemetry.addData("error", "%f", error);
        telemetry.addData("turretPower", "%f", robot.turret.getVelocity());
        telemetry.addData("Tx", "%f", tx);
        telemetry.update();


//        else{
//            double robotHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//            if (gamepad1.right_trigger > 0.1) {robot.imu.resetYaw();}
//            double turretAngle = robot.turret.getCurrentPosition()/turret_tPERd;
//            double target = normA(angleWant - robotHeading - tx);
//            if (target > 135) {target = 135;} else if (target < -135) {target = -135;}
//            double error = target - turretAngle;
//            double turretPower = clamp(error * slow, -1, 1);
////            turret.setPower(turretController.calculate(turretAngle, target));
////            robot.turret.setVelocity(turretController.calculate(turretAngle, target) * 1400 - robot.imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate * turret_tPERd);
//
////            robot.turret.setVelocity(turretController.calculate(turretAngle, target) * 1400 - imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate * turret_tPERd);
//
//
//            telemetry.addData("turretPos", "%d", robot.turret.getCurrentPosition());
//            telemetry.addData("robotHeading", "%f", robotHeading);
//            telemetry.addData("turretAngle", "%f", turretAngle);
//            telemetry.addData("turretTarget", "%f", target);
//            telemetry.addData("error", "%f", error);
//            telemetry.addData("turretPower", "%f", robot.turret.getVelocity());
//            telemetry.update();
//        }

        double distanceToGoal =  robot.limelight(ty, tx);
        double motorPower = robot.getshooterPower(distanceToGoal);
        double hoodAngle = robot.getHoodAngle(distanceToGoal);




        telemetry.addData("turretPower", "%f", robot.turret.getVelocity());
        telemetry.addData("tx", tx);
        telemetry.addData("front:", "%f", robot.intake.getPower());
        telemetry.addData("back:", "%f", robot.intake2.getPower());
        telemetry.update();

    }




    @Override
    public void stop(){
        robot.limelight.stop();

    }

    public double normA(double angle) {angle %= 360; if (angle < -180) angle += 360; else if (angle > 180) angle -= 360;return angle;}
    public double clamp(double x, double min, double max) {return Math.max(min,Math.min(max,x));}

}
