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
@TeleOp(name = "Mechanum-Red", group = "Linear OpMode")
public class HackinHounds_Mechanum_Red extends OpMode {

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
    private static double angleWant = -120;
    private static double slow = 1;

    public static double p = 0.02;
    public static double i = 0.002;
    public static double d = 0.004;
    public static double f = 0;

    PIDFController turretController = new PIDFController(p,i,d,f);


     public static double kp = 11;
     public static double ki = 0;
     public static double kd = 0;
     public static double kf = 0.8;

    PIDFController shooterController = new PIDFController(kp, ki, kd, kf);

    public static double hoodAngle = 0;
    public static double stopperPosition = .47;
    //position is 0.44 for stopping and 0.63 for neutral

    public static double shootertarget = 0;

    public static double target = 0;

    boolean isBlockerClosed = true;

    boolean rightBumper_pressed_previous = false;

    boolean leftBumper_pressed_previous = false;

    public static double shooterAngle = 0;

    private ElapsedTime stopperTimer = new ElapsedTime();
    private boolean isStopperTimedOpen = false;
    private boolean isSingleStopperTimedOpen = false;
    @Override
    public void init(){
        robot.init(hardwareMap);


        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        telemetry.addData("Status", "Initialized");
        telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.update();
        robot.limelight.pipelineSwitch(1);
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



        if (gamepad1.back) {
            robot.imu.resetYaw();
        }


        if(gamepad2.b){
            robot.intake.setPower(0.7);
            robot.intake2.setPower(-0.7);
        }

        if(gamepad2.y){
            robot.intake.setPower(0);
            robot.intake2.setPower(0);
        }

        if(gamepad2.a){
            robot.intake2.setPower(-0.3);
            robot.intake.setPower(0.3);
        }

        if(gamepad2.right_trigger > Math.abs(0.1)) {
            robot.intake.setPower(-0.7);
        }

        if(gamepad1.right_bumper){
            robot.intake.setPower(-0.7);
        }

        if (gamepad1.left_bumper) {
            angleWant = -120;
        }


        //Stopper logic for ONE BY ONE
        if(gamepad2.left_bumper && !leftBumper_pressed_previous) {
            if (isBlockerClosed) {
                robot.stopper.setPosition(0.47);
                isBlockerClosed = false;
                isSingleStopperTimedOpen = true;
                robot.intake.setPower(0.4);
                robot.intake2.setPower(-0.5);
                timer.reset();
            } else {
                robot.stopper.setPosition(0.7);
                isBlockerClosed = true;
                isSingleStopperTimedOpen = false;
                robot.intake.setPower(0.7);
                robot.intake2.setPower(-0.7);


            }
        }

        if(isSingleStopperTimedOpen && timer.seconds() > 0.16){
                robot.stopper.setPosition(0.7);
                isBlockerClosed = true;
                isSingleStopperTimedOpen = false;
                robot.intake.setPower(0.4);
                robot.intake2.setPower(-0.5);
        }




        // --- Stopper Control Logic to shoot ALL THREE

        // Check for 'A' press on gamepad2 (assuming gamepad2 for shooter controls)
        if (gamepad2.right_bumper && !rightBumper_pressed_previous) {
            // This block executes ONLY on the moment 'A' is pressed down

            if (isBlockerClosed) {
                // Stopper is currently closed -> Open it and start the timer
                robot.stopper.setPosition(0.47);// Open position
                robot.intake.setPower(0.5);
                robot.intake2.setPower(-0.5);
                isBlockerClosed = false;
                isStopperTimedOpen = true; // Signal that we are waiting for a close event
                stopperTimer.reset(); // Start the 1-second countdown

            } else {
                // Stopper is currently open -> Close it manually (if desired)
                robot.stopper.setPosition(0.7); // Closed position
                isBlockerClosed = true;
                isStopperTimedOpen = false; // Cancel any active timer wait
                robot.intake.setPower(0.7);
                robot.intake2.setPower(-0.7);
            }
        }

        // --- Automatic Close Check ---
        // If the stopper was opened by the timer logic AND 1.0 seconds have passed:
        if (isStopperTimedOpen && stopperTimer.seconds() >= 1) {
            // Close the stopper
            robot.stopper.setPosition(0.7);// Closed position
            robot.intake.setPower(0.7);
            robot.intake2.setPower(-0.7);
            isBlockerClosed = true;
            isStopperTimedOpen = false; // Stop checking the timer until the next press
        }

        // Update the previous state for the next loop
        rightBumper_pressed_previous = gamepad2.right_bumper;
        leftBumper_pressed_previous = gamepad2.left_bumper;








//
//
//        double target = normA( turretAngle + tx);
//        if (target > 135) {target = 135;} else if (target < -135) {target = -135;}
////        robot.turret.setPower(-turretController.calculate(turretAngle, target));
//
//

//        double robotYaw = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//
//// 1. Calculate the raw target angle the turret is commanded to go to (relative to robot)
//// This is the simplest tracking: current turret angle plus the error.
//
//        robot.turret.setPower(turretController.calculate(turretAngle, target));
//
//        double correctedOutput  = rawOutput * -1.0;
//        robot.turret.setPower(correctedOutput);



//        robot.angleServo.setPosition(hoodAngle);
//        if (gamepad2.right_bumper) {
//            startTime = timer.milliseconds();
//            robot.shooter.setVelocity(shooterPower);
//            stopped = false;
//        }
//        telemetry.addData("Since click", timer.milliseconds() - startTime);
//        if (robot.shooter.getVelocity() > shooterPower - 10 && !stopped) {
//            stopped = true;
//            finaltime = timer.milliseconds() - startTime;
//        } else {
//            telemetry.addData("time to spin up", finaltime);
//        }
//
//        if (gamepad2.left_bumper){stopped = false;}




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


        double shooterVelocity = robot.shooter.getVelocity();

        double output = shooterController.calculate(shooterVelocity, shootertarget);





        robot.angleServo.setPosition(shooterAngle);


        if(gamepad1.a){
            robot.shooter.setVelocity(0);
        } else{
            robot.shooter.setVelocity(output);

        }



        //distance >= 130 = 1.9 pffset

        double robotHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        //if (gamepad1.right_trigger > 0.1) {angleWant = robotHeading;}
        double turretAngle = robot.turret.getCurrentPosition()/turret_tPERd;
        if (result.isValid() && !gamepad1.left_bumper) {angleWant = (robotHeading + turretAngle) - tx;}

        double target = normA(angleWant - robotHeading);
        if (target > 150) {target = 150;} else if (target < -150) {target = -150;}
//        double error = target - turretAngle;
//        double turretPower = clamp(error * slow, -1, 1);
        robot.turret.setVelocity(turretController.calculate(turretAngle, target) * 1450 - robot.imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate * turret_tPERd);



        telemetry.addData("imu", "%f", robotHeading);

        telemetry.addData("turretPos", "%d", robot.turret.getCurrentPosition());
        telemetry.addData("turretAngle", "%f", turretAngle);
        telemetry.addData("turretTarget", "%f", target);
//        telemetry.addData("error", "%f", error);
        telemetry.addData("turretPower", "%f", robot.turret.getVelocity());
        telemetry.addData("Tx", "%f", tx);




        telemetry.addData("turretPower", "%f", robot.turret.getVelocity());
        telemetry.addData("tx", tx);
        telemetry.addData("front:", "%f", robot.intake.getPower());
        telemetry.addData("back:", "%f", robot.intake2.getPower());
        telemetry.addData("turretPos", "%f", robot.turret.getCurrentPosition()/4.233);
        telemetry.addData("turretTarget", "%f", target);
//        telemetry.addData("error", "%f", error);
        telemetry.addData("turretPower", "%f", robot.turret.getVelocity());
        telemetry.addData("distancetogoal", distanceToGoal);
        telemetry.addData("shooterPower", robot.shooter.getVelocity());
        telemetry.addData("targetShootPower", motorPower);

        telemetry.addData("1", mapColor(robot.color1.red(), robot.color1.green(), robot.color1.blue()));
        telemetry.addData("2", mapColor(robot.color2.red(), robot.color2.green(), robot.color2.blue()));
        telemetry.addData("3", mapColor(robot.color3.red(), robot.color3.green(), robot.color3.blue()));

        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("shooterVelocity", robot.shooter.getVelocity());
        packet.put("shooterRPM", (robot.shooter.getVelocity() / 28.0) * 60.0);
        packet.put("shootertarget", shootertarget);
        packet.put("turretTarget",  target);

        dashboard.sendTelemetryPacket(packet);

    }





    @Override
    public void stop(){
        robot.limelight.stop();

    }

    public double normA(double angle) {angle %= 360; if (angle < -180) angle += 360; else if (angle > 180) angle -= 360;return angle;}
    public double clamp(double x, double min, double max) {return Math.max(min,Math.min(max,x));}
    double mapColor(double r, double g, double b) {
        telemetry.addData("", "%f %f %f", r, g, b);

//        double max = Math.max(r, Math.max(g, b));
//        if (max > 0) {
//            r /= max;
//            g /= max;
//            b /= max;
//        }

        // Determine dominant channel
        boolean blueMax = b >= g && b >= r;
        boolean greenMax = g >= b && g >= r;

        // PURPLE: Blue is highest
        if (blueMax) {
            return 0.722; // violet
        }

        // GREEN: Green is highest
        if ((greenMax) && (g - b > 20)) {
            return 0.500; // green
        }

        // Neither
        return 0.000;
    }
}
