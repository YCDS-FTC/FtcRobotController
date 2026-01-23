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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.HackinHoundsHardware;


@Config
@TeleOp(name = "Color sensor demonstration", group = "Linear OpMode")
public class HackinHounds_Color_Sensor extends OpMode {

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


     public static double kp = 11;
     public static double ki = 0;
     public static double kd = 2;
     public static double kf = 1;

     public static double ks = 50;

    PIDFController shooterController = new PIDFController(kp, ki, kd, kf);

    public static double hoodAngle = 0;
    public static double stopperPosition = .47;
    //position is 0.44 for stopping and 0.63 for neutral

    public  static double shootertarget = 0;

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
        if (isStopperTimedOpen && stopperTimer.seconds() >= 1.5) {
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









        robot.lights(robot.light1, robot.light2, robot.light3, robot.color0, robot.color1, robot.color2, robot.color3);



        NormalizedRGBA colors0 = robot.color0.getNormalizedColors();

        int color0 = colors0.toColor();

        float[] hsvValues0 = new float[3];
        Color.colorToHSV(color0, hsvValues0);

        float hue0 = hsvValues0[0];
        float saturation0 = hsvValues0[1];



        NormalizedRGBA colors1 = robot.color1.getNormalizedColors();

        int color1 = colors1.toColor();

        float[] hsvValues1 = new float[3];
        Color.colorToHSV(color1, hsvValues1);

        float hue2 = hsvValues1[0];
        float saturation2 = hsvValues1[1];
        float value2 = hsvValues1[2];





        telemetry.addData("hue", hue0);
        telemetry.addData("saturation", saturation0);

        telemetry.addData("hue2", hue2);
        telemetry.addData("saturation2", saturation2);
        telemetry.addData("value2", value2);

        telemetry.addData("turretPos", "%d", robot.turret.getCurrentPosition());
        telemetry.addData("turretTarget", "%f", target);
        telemetry.addData("turretPower", "%f", robot.turret.getVelocity());
        telemetry.addData("Tx", "%f", tx);

        telemetry.addData("bumper variable", rightBumper_pressed_previous);
        telemetry.addData("stopperPos", robot.stopper.getPosition());
        telemetry.addData("stopperTimer", stopperTimer.seconds());



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
