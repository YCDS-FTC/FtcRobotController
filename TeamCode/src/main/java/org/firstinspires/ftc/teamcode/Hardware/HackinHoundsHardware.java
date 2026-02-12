package org.firstinspires.ftc.teamcode.Hardware;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.Rev9AxisImuOrientationOnRobot;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
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
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
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

    public Servo light1;
    public Servo light2;
    public Servo light3;
    public Servo light4;

    public NormalizedColorSensor color0, color1, color2, color3;

    public HuskyLens huskyLens;

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

    boolean notThere = true;


    private  final double think =  5.9;

    InterpLUT getShootPower = new InterpLUT();
    public static double shooterPower = 0;


    InterpLUT getShootPowerRed = new InterpLUT();

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
        //turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        stopper = robotMap.get(Servo.class,"stopper");
        angleServo = robotMap.get(Servo.class,"angleServo");


        light1 = robotMap.get(Servo.class,"light1");
        light2 = robotMap.get(Servo.class,"light2");
        light3 = robotMap.get(Servo.class,"light3");
        light4 = robotMap.get(Servo.class,"light4");


        color0 = robotMap.get(NormalizedColorSensor.class, "color0");
        color0.setGain(8);

        color1 = robotMap.get(NormalizedColorSensor.class,"color1");
        color1.setGain(8);
        color2 = robotMap.get(NormalizedColorSensor.class,"color2");
        color2.setGain(8);
        color3 = robotMap.get(NormalizedColorSensor.class,"color3");
        color3.setGain(8);

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

        huskyLens = robotMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);


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
        //imu.resetYaw();
        lastAngle = 0;


        getHoodAngle.add(0,0);
        getHoodAngle.add(57, 0.11);
        getHoodAngle.add(62, 0.1);
        getHoodAngle.add(67, 0.11);
        getHoodAngle.add(72, 0.13);
        getHoodAngle.add(77, 0.12);
        getHoodAngle.add(82, 0.14);
        getHoodAngle.add(87, 0.135);


        getHoodAngle.add(105,0.13);
        getHoodAngle.add(110,0.12);
        getHoodAngle.add(115,0.13);
        getHoodAngle.add(120, 0.125);
        getHoodAngle.add(125, 0.14);
        getHoodAngle.add(130, 0.14);
        getHoodAngle.add(148, .15);
        getHoodAngle.add(190, .1);








//add 18.5 inches away for offset/
        getShootPower.add(-20, 1000);
        getShootPower.add(1,1000);
        getShootPower.add(27, 1100);
        getShootPower.add(32, 1080);
        getShootPower.add(37, 1080);
        getShootPower.add(42, 1100);
        getShootPower.add(47, 1080);
        getShootPower.add(52, 1080);
        getShootPower.add(57, 1120);
        getShootPower.add(62, 1140);
        getShootPower.add(67, 1160);
        getShootPower.add(72, 1220);
        getShootPower.add(77, 1240);
        getShootPower.add(82, 1260);
        getShootPower.add(87, 1290);
        getShootPower.add(92, 1310);
        getShootPower.add(95,1420);
        getShootPower.add(100,1440);
        getShootPower.add(105,1440);
        getShootPower.add(110,1460);
        getShootPower.add(115,1500);
        getShootPower.add(120,1520);
        getShootPower.add(125,1520);
        getShootPower.add(130,1560);
        getShootPower.add(135,1580);
        getShootPower.add(148, 1580);
        getShootPower.add(250, 1600);




        getShootPowerRed.add(27, 950);
        getShootPowerRed.add(32, 1050);
        getShootPowerRed.add(37, 1050);
        getShootPowerRed.add(42, 1050);
        getShootPowerRed.add(47, 1060);
        getShootPowerRed.add(52, 1080);
        getShootPowerRed.add(57, 1120);
        getShootPowerRed.add(62, 1140);
        getShootPowerRed.add(67, 1160);
        getShootPowerRed.add(72, 1180);
        getShootPowerRed.add(77, 1220);
        getShootPowerRed.add(82, 1260);
        getShootPowerRed.add(87, 1290);
        getShootPowerRed.add(105,1440);
        getShootPowerRed.add(110,1460);
        getShootPowerRed.add(115,1500);
        getShootPowerRed.add(120,1520);
        getShootPowerRed.add(125,1520);
        getShootPowerRed.add(130,1520);
        getShootPowerRed.add(148, 1560);
        getShootPowerRed.add(190, 1560);





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


            shooterPower = getShootPower.get(distanceToGoal);


        return shooterPower;
    }

    public double getshooterPowerRed(double distanceToGoal) {
        getShootPowerRed.createLUT();

        if(distanceToGoal > 190){
            shooterPower = 0;
        } else if (distanceToGoal < 27){
            shooterPower = 0;
        } else{
            shooterPower = getShootPowerRed.get(distanceToGoal);
        }

        return shooterPower;
    }

    public double getHoodAngle (double distanceToGoal) {
        getHoodAngle.createLUT();

        if (distanceToGoal > 190) {
            hoodAngle = 0;
        } else if (distanceToGoal < 27) {
            hoodAngle = 0;
        } else{
            hoodAngle = getHoodAngle.get(distanceToGoal);

        }

        return hoodAngle;

    }

    public double odometryDistanceBlue(double X, double Y){

        double distanceToGoal1 = Math.sqrt(Math.pow(0 - X, 2) + Math.pow(144 - Y, 2));

        return distanceToGoal1 - 26 ;
    }

    public double odometryDistanceRed(double X, double Y){

        double distanceToGoal1 = Math.sqrt(Math.pow(144 - X, 2) + Math.pow(144 - Y, 2));

        return distanceToGoal1 - 24.5;
    }


    public double limelight(double ty, double tx){
        double targetOffsetAngle_Vertical = ty;
        double txToRadians = tx * (3.14159/180.0);
        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159/ 180.0);

        double distanceToGoal1 = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians) + 3.3;
        double distanceToGoal = (distanceToGoal1) * Math.cos(txToRadians);

        return distanceToGoal;
    }


    public double mapColor(double r, double g, double b) {
        boolean blueMax = b >= g && b >= r;
        boolean greenMax = g >= b && g >= r;
        if (blueMax) {
            return 0.722;
        }
        if ((greenMax) && (g - b > 20)) {
            return 0.500;
        }
        return 0.000;
    }




    public void lights(Servo light1, Servo light2, Servo light3, NormalizedColorSensor ncolor0, NormalizedColorSensor ncolor1, NormalizedColorSensor ncolor2, NormalizedColorSensor ncolor3) {

        NormalizedRGBA colors0 = ncolor0.getNormalizedColors(), colors1 = ncolor1.getNormalizedColors(), colors2 = ncolor2.getNormalizedColors(), colors3 = ncolor3.getNormalizedColors();

        int color0 = colors0.toColor(), color1 = colors1.toColor(), color2 = colors2.toColor(), color3 = colors3.toColor();
        int green0 = Color.green(color0), blue0 = Color.blue(color0);
        int green1 = Color.green(color1), blue1 = Color.blue(color1);

        float[] hsvValues1 = new float[3];
        Color.colorToHSV(color1, hsvValues1);
        float[] hsvValues2 = new float[3];
        Color.colorToHSV(color2, hsvValues2);
        float[] hsvValues3 = new float[3];
        Color.colorToHSV(color3, hsvValues3);
        float[] hsvValues0 = new float[3];
        Color.colorToHSV(color0, hsvValues0);

        float hue0 = hsvValues0[0], hue1 = hsvValues1[0], hue2 = hsvValues2[0], hue3 = hsvValues3[0];
        float saturation0 = hsvValues0[1], saturation2 = hsvValues2[1], saturation3 = hsvValues3[1];

        if(hue3 < 150 && hue3 != 120){
            light3.setPosition(0.5);
            notThere = false;
        } else if(hue3 > 180){
            light3.setPosition(0.722);
            notThere = false;
        } else{
            light3.setPosition(0);
            notThere = true;
        }
        if(saturation2 > 0.69 || hue2 > 120 && hue2 < 169 ){
            light2.setPosition(0.5);
        } else if(saturation2 < 0.5 && hue2 > 170){
            light2.setPosition(0.722);
        } else{
            light2.setPosition(0);
        }

        if (notThere){
            if (hue2 == 150){
                light2.setPosition(0);
            }
        }
        if (saturation0 > 0.5 && (green0 > blue0 || green1 > blue1)){
            light1.setPosition(0.5);
        } else if(hue1 != 180 && hue1!= 150){
            light1.setPosition(0.722);
        } else{
            light1.setPosition(0);
        }
    }
}

