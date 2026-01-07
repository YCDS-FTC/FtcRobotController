package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
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


@Config
@TeleOp(name = "Defence-Bot", group = "Linear OpMode")
public class Mecanum_Testing extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();

    private FtcDashboard dashboard;

    double shift = 1;

    double startTime = 0;
    boolean stopped = false;
    double finaltime = 0;


    public static double transferPower = 0;
    public static double intakePower = 0;

    public DcMotorEx leftFront;
    public DcMotorEx  rightFront;
    public DcMotorEx  leftBack;
    public DcMotorEx  rightBack;

    public double lastAngle;
    public IMU imu;
    public YawPitchRollAngles angles;

    private YawPitchRollAngles lastAngles;
    private double globalAngle;


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



        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        telemetry.addData("Status", "Initialized");
        telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.update();


        leftFront  = hardwareMap.get(DcMotorEx.class, "left_front");
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front");
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack = hardwareMap.get(DcMotorEx.class, "left_back");
        rightBack = hardwareMap.get(DcMotorEx.class, "right_back");
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        imu = hardwareMap   .get(IMU.class, "imu");
    }

    @Override
    public void start(){

    }


    @Override
    public void loop(){





        if (gamepad1.dpad_up) {
            shift = 1;
        } else if (gamepad1.dpad_right) {
            shift = 0.75;
        } else if (gamepad1.dpad_down) {
            shift = 0.5;
        }

        double facing = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
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

        leftBack.setVelocity(3000 * lb * shift);
        leftFront.setVelocity(3000 * lf * shift);
        rightBack.setVelocity(3000 * rb * shift);
        rightFront.setVelocity(3000 * rf * shift);
//
//        robot.intake.setPower(intakePower);
//        robot.intake2.setPower(transferPower);


    }




    @Override
    public void stop(){

    }

    public double normA(double angle) {angle %= 360; if (angle < -134) angle += 360; else if (angle > 134) angle -= 360;return angle;}
    public double clamp(double x, double min, double max) {return Math.max(min,Math.min(max,x));}

}
