package org.firstinspires.ftc.teamcode.TeleOp;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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


    private double filteredVar = 0;
    private boolean filterStart = false;
    private double filterTick = 0;

    // intake = 0.5
    // transfer = 0.8


    private static double turret_tPERd = 4.233;
    private static double angleWant = -120;
    private static double slow = 1;

    public static double p = 0.03;
    public static double i = 0;
    public static double d = 0.0035;
    public static double f = 0;

    PIDFController turretController = new PIDFController(p,i,d,f);


     public double kp = 11;
     public double ki = 0;
     public double kd = 0;
     public double kf = 0.8;

    PIDFController shooterController = new PIDFController(kp, ki, kd, kf);

    public static double hoodAngle = 0;
    public static double stopperPosition = .47;
    //position is 0.44 for stopping and 0.63 for neutral

    public  double shootertarget = 0;

    public  double target = 0;

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
        if (isStopperTimedOpen && stopperTimer.seconds() >= 2) {
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








        double distanceToGoal =  robot.limelight(ty, tx);

        if (Math.abs(filteredVar - distanceToGoal) > 50 && filterStart) {
            filterTick++;
//            distanceToGoal = filteredVar;
            if (filterTick < 10) {
                distanceToGoal = filteredVar;
            } else {
                filterTick = 0;
            }
        } else {
            filterTick = 0;
        }
        filteredVar = distanceToGoal;
        filterStart = true;
        double motorPower = robot.getshooterPowerRed(distanceToGoal);
        double hoodAngle = robot.getHoodAngle(distanceToGoal);


        double shooterVelocity = robot.shooter.getVelocity();

        double output = shooterController.calculate(shooterVelocity, motorPower);





        robot.angleServo.setPosition(hoodAngle);


        if(gamepad1.a){
            robot.shooter.setVelocity(0);
        } else{
            robot.shooter.setVelocity(output);

        }



        //distance >= 130 = 1.9 pffset

        double robotHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        //if (gamepad1.right_trigger > 0.1) {angleWant = robotHeading;}
        double turretAngle = robot.turret.getCurrentPosition()/turret_tPERd;

        if (result.isValid() && !gamepad1.left_bumper && distanceToGoal > 100){
            angleWant = (robotHeading + turretAngle) - tx - 1.5;
        } else if (result.isValid() && !gamepad1.left_bumper) {
            angleWant = (robotHeading + turretAngle) - tx;
        }

        double target = normA(angleWant - robotHeading);
        if (target > 150) {target = 150;} else if (target < -150) {target = -150;}
//        double error = target - turretAngle;
//        double turretPower = clamp(error * slow, -1, 1);
        robot.turret.setVelocity(turretController.calculate(turretAngle, target) * 1450 - robot.imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate * turret_tPERd);



//        robot.light1.setPosition(mapColor(robot.color1.getNormalizedColors().red, robot.color1.getNormalizedColors().green, robot.color1.getNormalizedColors().blue));
//        robot.light2.setPosition(mapColor(robot.color2.getNormalizedColors().red, robot.color2.getNormalizedColors().green, robot.color2.getNormalizedColors().blue));
//        robot.light3.setPosition(mapColor(robot.color3.getNormalizedColors().red, robot.color3.getNormalizedColors().green, robot.color3.getNormalizedColors().blue));
//        robot.light3.setPosition(mapColor(robot.color4.getNormalizedColors().red, robot.color4.getNormalizedColors().green, robot.color4.getNormalizedColors().blue));


        NormalizedRGBA colors0 = robot.color0.getNormalizedColors();
        NormalizedRGBA colors1 = robot.color1.getNormalizedColors();
        NormalizedRGBA colors2 = robot.color2.getNormalizedColors();
        NormalizedRGBA colors3 = robot.color3.getNormalizedColors();

        int color0 = colors0.toColor();
        int color1 = colors1.toColor();
        int color2 = colors2.toColor();
        int color3 = colors3.toColor();

        int red0 = Color.red(color0);
        int green0 = Color.green(color0);
        int blue0 = Color.blue(color0);

        int red1 = Color.red(color1);
        int green1 = Color.green(color1);
        int blue1 = Color.blue(color1);

        int red2 = Color.red(color2);
        int green2 = Color.green(color2);
        int blue2 = Color.blue(color2);

        int red3 = Color.red(color3);
        int green3 = Color.green(color3);
        int blue3 = Color.blue(color3);



        float[] hsvValues1 = new float[3];
        Color.colorToHSV(color1, hsvValues1);


        float[] hsvValues2 = new float[3];
        Color.colorToHSV(color2, hsvValues2);


        float[] hsvValues3 = new float[3];
        Color.colorToHSV(color3, hsvValues3);

        float[] hsvValues0 = new float[3];
        Color.colorToHSV(color0, hsvValues0);


        float hue3 = hsvValues3[0];        // 0-360
        float saturation3 = hsvValues3[1]; // 0-1
        float value3 = hsvValues3[2];// 0-1


        float hue2 = hsvValues2[0];        // 0-360
        float saturation2 = hsvValues2[1]; // 0-1
        float value2 = hsvValues2[2];

        float hue1 = hsvValues1[0];        // 0-360
        float saturation1 = hsvValues1[1]; // 0-1
        float value1 = hsvValues1[2];

        float hue0 = hsvValues0[0];        // 0-360
        float saturation0 = hsvValues0[1]; // 0-1
        float value0 = hsvValues0[2];

// Detect color based on hue
        String detectedColor = "Unknown";


        if(saturation3 > 0.5 &&  140< hue3 && hue3 < 175){
            robot.light3.setPosition(0.5);
            detectedColor = "Green";
        } else if(saturation3 < 0.5 && hue3 > 180){
            robot.light3.setPosition(0.722);
            detectedColor = "Purple";
        } else{
            robot.light3.setPosition(0);
            detectedColor = "Purple";
        }

        if(saturation2 > 0.5){
            robot.light2.setPosition(0.5);
            detectedColor = "Green";
        } else if(saturation2 < 0.5 && hue2 > 170){
            robot.light2.setPosition(0.722);
            detectedColor = "Purple";
        } else{
            robot.light2.setPosition(0);
        }

        if (saturation0 > 0.5 && (green0 > blue0 || green1 > blue1)){
            robot.light1.setPosition(0.5);
        } else if(hue1 != 150 && hue0 != 150){
            robot.light1.setPosition(0.722);
        } else{
            robot.light1.setPosition(0);
        }
        telemetry.addData("", "%d, %d, %d", red0, green0, blue0);
        telemetry.addData("", "%d, %d, %d", red1, green1, blue1);



        telemetry.addData("Detected Color", detectedColor);
        telemetry.addData("Hue", hue1);
        telemetry.addData("Saturation", saturation1);
        telemetry.addData("Value", value1);
        telemetry.addData("Detected Color", detectedColor);
        telemetry.addData("Hue", hue0);
        telemetry.addData("Saturation", saturation0);
        telemetry.addData("Value", value0);
        telemetry.update();


//        if(robot.color1.getDistance(DistanceUnit.INCH) < 2) {
//            if (robot.light3.getPosition() == 0.722) {
//                robot.light1.setPosition(0.5);
//            } else {
//                robot.light1.setPosition(0.722);
//            }
//        } else {
//            robot.light1.setPosition(0);
//
//        }



        telemetry.addData("imu", "%f", robotHeading);

        telemetry.addData("turretPos", "%d", robot.turret.getCurrentPosition());
        telemetry.addData("turretAngle", "%f", turretAngle);
        telemetry.addData("turretTarget", "%f", target);
//        telemetry.addData("error", "%f", error);
        telemetry.addData("turretPower", "%f", robot.turret.getVelocity());
        telemetry.addData("Tx", "%f", tx);


        telemetry.addData("bumper variable", rightBumper_pressed_previous);
        telemetry.addData("stopperPos", robot.stopper.getPosition());
        telemetry.addData("stopperTimer", stopperTimer.seconds());




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

        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("shooterVelocity", robot.shooter.getVelocity());
        packet.put("shooterRPM", (robot.shooter.getVelocity() / 28.0) * 60.0);
        packet.put("shootertarget", shootertarget);
        packet.put("turretTarget",  target);
        packet.put("turretPos", turretAngle);
        dashboard.sendTelemetryPacket(packet);

    }





    @Override
    public void stop(){
        robot.limelight.stop();

    }

    public double normA(double angle) {angle %= 360; if (angle < -180) angle += 360; else if (angle > 180) angle -= 360;return angle;}
    public double clamp(double x, double min, double max) {return Math.max(min,Math.min(max,x));}


    double mapColor(double r, double g, double b) {
        telemetry.addData("", "%f, %f, %f", r, g, b);
        boolean blueMax = b >= g && b >= r;
        boolean greenMax = g >= b && g >= r;
        if (blueMax) {
            return 0.722;
        }
        if ((greenMax) && (g - b > 0.0005)) {
            return 0.500;
        }
        return 0.000;
    }
}
