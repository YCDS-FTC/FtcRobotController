package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.Rev9AxisImuOrientationOnRobot;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.openftc.easyopencv.OpenCvCamera;

// Generic robot class
public class HackinHoundsHardware extends Hardware {
    public HardwareMap robotMap;

    /** sensors **/
//    public AnalogInput test1, test2, test3;
//    public Servo light, light2;
    public Limelight3A limelight;

    /** Drivetrain Members **/
    public DcMotorEx  leftFront;
    public DcMotorEx  rightFront;
    public DcMotorEx  leftBack;
    public DcMotorEx  rightBack;

    /** intake members **/
    public DcMotorEx intake;
    public DcMotorEx intake2;



    /** outake members **/
    public DcMotorEx shooter;
    public Servo angleServo;
    public Servo stopper;
    public DcMotorEx turret;



    public GoBildaPinpointDriver pinpoint;

    public  VoltageSensor voltageSensor;

    public double lastAngle;
    public IMU imu;
    public YawPitchRollAngles angles;

    private YawPitchRollAngles lastAngles;
    private double globalAngle;

    // 1000 ticks was roughly 18 in.
    public static final double TICK_PER_INCH = 432/24;
    public static final double MinPower = 0.1;


    private  final double think =  5.9;

    InterpLUT getShootPower = new InterpLUT();
    public static double shooterPower = 0;

    InterpLUT getHoodAngle = new InterpLUT();
    public static double hoodAngle = 0;



    // how many degrees back isA your limelight rotated from perfectly vertical?
    public double limelightMountAngleDegrees = 20.0;

    // distance from the center of the Limelight lens to the floor
    public double limelightLensHeightInches =  14.0;

    // distance from the target to the floor
    public double goalHeightInches = 29.5;



    /**
     GLOBAL VARIABLES
     **/


    /* Constructor */
    public HackinHoundsHardware(){

    }

    // Override to set actual robot configuration
    public void init(HardwareMap hwMap) {

        // Save reference to Hardware map
        robotMap = hwMap;

//        limelight = robotMap.get(Limelight3A.class, "limelight");


        // Define and Initialize Motors for drivetrain
        leftFront  = robotMap.get(DcMotorEx.class, "leftFront");
        rightFront = robotMap.get(DcMotorEx.class, "rightFront");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack = robotMap.get(DcMotorEx.class, "leftBack");
        rightBack = robotMap.get(DcMotorEx.class, "rightBack");
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        shooter = robotMap.get(DcMotorEx.class,"shooter");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turret = robotMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        stopper = robotMap.get(Servo.class,"stopper");
        angleServo = robotMap.get(Servo.class,"angleServo");



        limelight = robotMap.get(Limelight3A.class, "limelight");

        pinpoint = robotMap.get(GoBildaPinpointDriver.class, "pinpoint");

        //leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        voltageSensor = robotMap.get(VoltageSensor.class, "Control Hub");


        intake = robotMap.get(DcMotorEx.class,"intake");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake2 = robotMap.get(DcMotorEx.class,"intake2");
        intake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        // Defines the REV Hub's internal IMU (Gyro)
        imu = robotMap.get(IMU.class, "imu");

        //imu.resetYaw();

        Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection usb = Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection.LEFT;
        Rev9AxisImuOrientationOnRobot.LogoFacingDirection logo = Rev9AxisImuOrientationOnRobot.LogoFacingDirection.DOWN;

//        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
//        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
//        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        Rev9AxisImuOrientationOnRobot orientationOnRobot = new Rev9AxisImuOrientationOnRobot(logo, usb);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        lastAngle = 0;


        getHoodAngle.add(0,0);
        getHoodAngle.add(117,0.18);
        getHoodAngle.add(123,0.18);
        getHoodAngle.add(128, 0.18);





        getShootPower.add(23, 1220);
        getShootPower.add(28, 1240);
        getShootPower.add(33, 1280);
        getShootPower.add(38, 1260);
        getShootPower.add(43, 1260);
        getShootPower.add(48, 1260);
        getShootPower.add(53, 1280);
        getShootPower.add(58, 1300);
        getShootPower.add(63, 1320);
        getShootPower.add(68, 1350);
        getShootPower.add(73, 1360);
        getShootPower.add(78, 1260);
        getShootPower.add(83,1280);
        getShootPower.add(117,1540);
        getShootPower.add(123, 1580);
        getShootPower.add(128, 1620);


        //imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public double clamp(double x, double min, double max) {
        return Math.max(min,Math.min(max,x));
    }

    //
    public double getAngle() {
        double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        double deltaAngle = angle - lastAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngle = angle;

        return globalAngle;


    }

    public double getDistance (AnalogInput sensor) {
        return (sensor.getVoltage() * 48.7) - 4.9;
    }

    public double getshooterPower(double distanceToGoal) {
        getShootPower.createLUT();

        if(distanceToGoal > 128){
            shooterPower = 0;
        } else {
            shooterPower = getShootPower.get(distanceToGoal);
        }

        return shooterPower;
    }

    public double getHoodAngle (double distanceToGoal) {
        getHoodAngle.createLUT();

        if (distanceToGoal > 128) {
            hoodAngle = 0;
        } else if (distanceToGoal < 117) {
            hoodAngle = 0;
        } else{
            hoodAngle = getHoodAngle.get(distanceToGoal);

        }

        return hoodAngle;

    }

    public double limelight(double ty, double tx){

         double targetOffsetAngle_Vertical = ty;
         double txToRadians = tx * (3.14159 / 180.0);
         double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
         double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        double distanceToGoal1 = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians) + 3.3;


        double distanceToGoal = (distanceToGoal1) * Math.cos(txToRadians);
        return distanceToGoal;
    }

}

