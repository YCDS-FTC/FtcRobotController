package org.firstinspires.ftc.teamcode.TeleOp;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.HackinHoundsHardware;


@Config
@TeleOp(name = "RESET", group = "Linear OpMode")
public class Reset extends OpMode {

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
    public void loop(){
        robot.imu.resetYaw();
        robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double normA(double angle) {angle %= 360; if (angle < -180) angle += 360; else if (angle > 180) angle -= 360;return angle;}
    public double clamp(double x, double min, double max) {return Math.max(min,Math.min(max,x));}

}
