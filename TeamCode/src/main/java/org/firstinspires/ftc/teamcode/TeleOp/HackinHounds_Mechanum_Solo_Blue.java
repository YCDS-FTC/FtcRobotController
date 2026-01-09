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
@TeleOp(name = "Mechanum-Solo-Blue", group = "Linear OpMode")
public class HackinHounds_Mechanum_Solo_Blue extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();
    private HackinHoundsHardware robot = new HackinHoundsHardware();

    private FtcDashboard dashboard;

    double shift = 1;

    double startTime = 0;
    boolean stopped = false;
    double finaltime = 0;


    public  double transferPower = 0;
    public  double intakePower = 0;



    // intake = 0.5
    // transfer = 0.8


    private  double turret_tPERd = 4.233;
    private  double angleWant = 120;
    private  double slow = 1;

    public static double p = 0.05;
    public static double i = 0;
    public static double d = 0.006;
    public static double f = 0;

    PIDFController turretController = new PIDFController(p,i,d,f);


    public static double kp = 11;
    public static double ki = 0;
    public static double kd = 2;
    public static double kf = 1;

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

    private double lastValidDistance = Double.NaN;

    private double MAX_DISTANCE_DELTA = 15;
    public static double turretoffset = 0;

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

        if (gamepad1.back) {
            robot.imu.resetYaw();
        }


        if(gamepad1.b){
            robot.intake.setPower(0.7);
            robot.intake2.setPower(-0.7);
        }

        if(gamepad1.y){
            robot.intake.setPower(0);
            robot.intake2.setPower(0);
        }

        if(gamepad1.a){
            robot.intake2.setPower(-0.3);
            robot.intake.setPower(0.3);
        }

        if(gamepad1.right_trigger > Math.abs(0.1)) {
            robot.intake.setPower(-0.7);
        }

        if (gamepad1.left_bumper) {
            angleWant = 120;
        }


        //Stopper logic for ONE BY ONE
        if(gamepad1.left_bumper && !leftBumper_pressed_previous) {
            if (isBlockerClosed) {
                robot.stopper.setPosition(0.47);
                isBlockerClosed = false;
                isSingleStopperTimedOpen = true;
                robot.intake.setPower(0.3);
                robot.intake2.setPower(-0.3);
                timer.reset();
            } else {
                robot.stopper.setPosition(0.67);
                isBlockerClosed = true;
                isSingleStopperTimedOpen = false;
                robot.intake.setPower(0.7);
                robot.intake2.setPower(-0.7);


            }
        }

            if(isSingleStopperTimedOpen && timer.seconds() > 0.2){
                robot.stopper.setPosition(0.7);
                isBlockerClosed = true;
                isSingleStopperTimedOpen = false;
                robot.intake.setPower(0.4);
                robot.intake2.setPower(-0.6);
            }




        // --- Stopper Control Logic to shoot ALL THREE

        // Check for 'A' press on gamepad2 (assuming gamepad2 for shooter controls)
        if (gamepad1.right_bumper && !rightBumper_pressed_previous) {
            // This block executes ONLY on the moment 'A' is pressed down

            if (isBlockerClosed) {
                // Stopper is currently closed -> Open it and start the timer
                robot.stopper.setPosition(0.47);// Open position
                robot.intake.setPower(0.3);
                robot.intake2.setPower(-0.3);
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
            robot.stopper.setPosition(0.67);// Closed position
            robot.intake.setPower(0.7);
            robot.intake2.setPower(-0.7);
            isBlockerClosed = true;
            isStopperTimedOpen = false; // Stop checking the timer until the next press
        }

        // Update the previous state for the next loop
        rightBumper_pressed_previous = gamepad2.right_bumper;
        leftBumper_pressed_previous = gamepad2.left_bumper;











        double distanceToGoal =  robot.limelight(ty, tx);

        if (Double.isNaN(lastValidDistance)) {
            // First reading — always accept it
            lastValidDistance = distanceToGoal;
        } else {
            double delta = Math.abs(distanceToGoal - lastValidDistance);

            if (delta <= MAX_DISTANCE_DELTA) {
                // Reading is reasonable → accept it
                lastValidDistance = distanceToGoal;
            }
            // else: reject the measurement and keep the old value
        }

        double filteredDistance = lastValidDistance;


        double motorPower = robot.getshooterPower(filteredDistance);
        double hoodAngle = robot.getHoodAngle(filteredDistance);




        double shooterVelocity = robot.shooter.getVelocity();

        double output = shooterController.calculate(shooterVelocity, motorPower);

        if(gamepad1.a){
            robot.shooter.setVelocity(0);
        } else{
            robot.angleServo.setPosition(hoodAngle);

        }


//        robot.shooter.setVelocity(motorPower);



        double robotHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        //if (gamepad1.right_trigger > 0.1) {angleWant = robotHeading;}
        double turretAngle = robot.turret.getCurrentPosition()/turret_tPERd;

        if (result.isValid() && !gamepad1.left_bumper && distanceToGoal > 100){
            angleWant = (robotHeading + turretAngle) - tx + 3.8;
        } else if (result.isValid() && !gamepad1.left_bumper) {
            angleWant = (robotHeading + turretAngle) - tx;
        }

        double target = normA(angleWant - robotHeading);
        if (target > 150) {target = 150;} else if (target < -150) {target = -150;}
//        double error = target - turretAngle;
//        double turretPower = clamp(error * slow, -1, 1);
        robot.turret.setVelocity(turretController.calculate(turretAngle, target) * 1450 - robot.imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate * turret_tPERd);

        robot.lights(robot.light1, robot.light2, robot.light3, robot.color0, robot.color1, robot.color2, robot.color3);


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

}
