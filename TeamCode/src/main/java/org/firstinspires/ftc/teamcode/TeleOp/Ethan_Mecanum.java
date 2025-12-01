package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.HackinHoundsHardware;


@Config
@TeleOp(name = "Ethan", group = "Linear OpMode")
public class Ethan_Mecanum extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();
    private HackinHoundsHardware robot = new HackinHoundsHardware();

    private FtcDashboard dashboard;

    double shift = 1;

    double startTime = 0;
    boolean stopped = false;
    double finaltime = 0;


    public static double transferPower = 0;
    public static double intakePower = 0;



    // intake = 0.5
    // transfer = 0.8


    private static double turret_tPERd = 4.233;
    private static double angleWant = 0;
    private static double slow = 1;

    public static double p = 0.02;
    public static double i = 0.002;
    public static double d = 0.0005;
    public static double f = 0;

    PIDFController turretController = new PIDFController(p,i,d,f);


    public static double kp = 0.02;
    public static double ki = 0.002;
    public static double kd = 0.0005;
    public static double kf = 0;

    PIDFController shooterController = new PIDFController(kp, ki, kd, kf);

    public static double hoodAngle = 0;
    public static double stopperPosition = .47;
    //position is 0.44 for stopping and 0.63 for neutral

    public static double shooterPower = 0;


    boolean isBlockerClosed = true;

    boolean a_pressed_previous = false;
    @Override
    public void init(){


        robot.init(hardwareMap);

        robot.stopper.setPosition(0.67);

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

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
        shooterController.setPIDF(kp,ki,kd,kf);


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
//
//        robot.intake.setPower(intakePower);
//        robot.intake2.setPower(transferPower);


        if(gamepad1.right_trigger > Math.abs(0.1)){
            robot.intake.setPower(0.5);
        } else{
            robot.intake.setPower(0);
        }


        if(gamepad1.left_trigger > Math.abs(0.1)){
            robot.intake2.setPower(-0.7);
        } else{
            robot.intake2.setPower(0);
        }

        if(gamepad1.right_bumper){
            robot.shooter.setVelocity(1240);
        }

        if(gamepad1.rightBumperWasReleased()){
            robot.shooter.setVelocity(0);
        }

//
//        if(gamepad1.a){
//            robot.stopper.setPosition(0.47);
//            isBlockerClosed = false;
//        } else if(gamepad1.a && !isBlockerClosed){
//            robot.stopper.setPosition(.67);
//            isBlockerClosed = true;
//        }

        if (gamepad1.a && !a_pressed_previous) {
            // This block executes ONLY on the moment 'A' is pressed down
            if (isBlockerClosed) {
                // Stopper is currently closed -> Open it
                robot.stopper.setPosition(0.47);
                isBlockerClosed = false;
            } else {
                // Stopper is currently open -> Close it
                robot.stopper.setPosition(0.67);
                isBlockerClosed = true;
            }
        }




        if (gamepad1.back) {
            robot.imu.resetYaw();
        }





        double turretAngle = robot.turret.getCurrentPosition()/turret_tPERd;
        double target = normA( turretAngle + tx);
        if (target > 135) {target = 135;} else if (target < -135) {target = -135;}
        double error = target - turretAngle;
        double turretPower = clamp(error * slow, -1, 1);
        robot.turret.setPower(turretController.calculate(turretAngle, target));





        robot.angleServo.setPosition(hoodAngle);
        if (gamepad2.right_bumper) {
            startTime = timer.milliseconds();
            robot.shooter.setVelocity(shooterPower);
            stopped = false;
        }
        telemetry.addData("Since click", timer.milliseconds() - startTime);
        if (robot.shooter.getVelocity() > shooterPower - 10 && !stopped) {
            stopped = true;
            finaltime = timer.milliseconds() - startTime;
        } else {
            telemetry.addData("time to spin up", finaltime);
        }

        if (gamepad2.left_bumper){stopped = false;}




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

            //        }

        double distanceToGoal =  robot.limelight(ty, tx);
        double motorPower = robot.getshooterPower(distanceToGoal);
        double hoodAngle = robot.getHoodAngle(distanceToGoal);



        a_pressed_previous = gamepad1.a;


        telemetry.addData("turretPower", "%f", robot.turret.getVelocity());
        telemetry.addData("tx", tx);
        telemetry.addData("front:", "%f", robot.intake.getPower());
        telemetry.addData("back:", "%f", robot.intake2.getPower());
        telemetry.addData("turretPos", "%f", robot.turret.getCurrentPosition()/4.233);
//        telemetry.addData("turretAngle", "%f", turretAngle);
//        telemetry.addData("turretTarget", "%f", target);
//        telemetry.addData("error", "%f", error);
        telemetry.addData("turretPower", "%f", robot.turret.getVelocity());
        telemetry.addData("shooterPower", "%f", robot.shooter.getVelocity());

        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("shooterVelocity", robot.shooter.getVelocity());
        packet.put("shooterRPM", (robot.shooter.getVelocity() / 28.0) * 60.0);

        dashboard.sendTelemetryPacket(packet);

    }




    @Override
    public void stop(){
        robot.limelight.stop();

    }

    public double normA(double angle) {angle %= 360; if (angle < -134) angle += 360; else if (angle > 134) angle -= 360;return angle;}
    public double clamp(double x, double min, double max) {return Math.max(min,Math.min(max,x));}

}
