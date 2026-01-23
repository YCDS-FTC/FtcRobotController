package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.HackinHoundsHardware;


@Config
//@TeleOp(name = "ColoeSense", group = "Linear OpMode")
public class ColorSense extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();
    private HackinHoundsHardware robot = new HackinHoundsHardware();

    private FtcDashboard dashboard;

    double shift = 1;

    private HuskyLens huskyLens;

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

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        robot.pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        robot.pinpoint.setOffsets(-3.4649606, -3.4649606, DistanceUnit.INCH);
        robot.pinpoint.setPosX(72, DistanceUnit.INCH);
        robot.pinpoint.setPosY(72, DistanceUnit.INCH);
        robot.pinpoint.update();

        turretController.setPIDF(p,i,d,f);
    }
    @Override
    public void start(){robot.limelight.start();}
    @Override
    public void loop(){

        LLResult result = robot.limelight.getLatestResult();

        double tx = result.getTx();
        double ty = result.getTy();

        double facing = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double y  = -gamepad1.left_stick_y;
        double x  = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if (gamepad1.left_bumper) {
            HuskyLens.Block[] blocks = huskyLens.blocks();
            HuskyLens.Block largest = null;
            int largestArea = 0;
            for (HuskyLens.Block b : blocks) {
                int area = b.width * b.height;
                if (area > largestArea) {
                    largestArea = area;
                    largest = b;
                }
            }

            double autoSteer = 0;
            if (largest != null) {
                double dis = (largest.x - 160) / 160.0;
//                autoSteer = dis * 0.35;
//                rx -= autoSteer;
                double assistPower = dis * 0.75;
                double angle = facing + (Math.PI / 2.0);
                y += assistPower * Math.cos(angle);
                x -= assistPower * Math.sin(angle);
            }
        }


        double rotX = x * Math.cos(-facing) - y * Math.sin(-facing);
        rotX *= 1.1;
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

        if(gamepad2.b){
            robot.intake.setPower(0.7);
            robot.intake2.setPower(-0.7);
        }

        if(gamepad2.y){
            robot.intake.setPower(0);
            robot.intake2.setPower(0);
        }



        double robotHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        robot.pinpoint.setHeading(robotHeading, AngleUnit.DEGREES);
        robot.pinpoint.update();
        //if (gamepad1.right_trigger > 0.1) {angleWant = robotHeading;}
        double turretAngle = robot.turret.getCurrentPosition()/turret_tPERd;
        //if (result.isValid() && !gamepad1.left_bumper) {angleWant = (robotHeading + turretAngle) - tx;}
        angleWant = Math.toDegrees(Math.atan2(140 - robot.pinpoint.getPosY(DistanceUnit.INCH), 130 - robot.pinpoint.getPosX(DistanceUnit.INCH))) - 180;
        //angleWant = Math.toDegrees(Math.atan2(140 - (robot.pinpoint.getPosY(DistanceUnit.INCH) + robot.pinpoint.getVelY(DistanceUnit.INCH)/2), 138 - (robot.pinpoint.getPosX(DistanceUnit.INCH) + robot.pinpoint.getVelX(DistanceUnit.INCH)/2))) - 180;
        double target = normA(angleWant - robotHeading);
        if (target > 150) {target = 150;} else if (target < -150) {target = -150;}
        //double error = target - turretAngle;
        //double turretPower = clamp(error * slow, -1, 1);
        robot.turret.setVelocity(turretController.calculate(turretAngle, target) * 1450 - robot.imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate * turret_tPERd);


        double distanceToGoal =  robot.limelight(ty, tx);
        //distanceToGoal = Math.sqrt(Math.pow(140 - (robot.pinpoint.getPosY(DistanceUnit.INCH) + robot.pinpoint.getVelY(DistanceUnit.INCH)/2), 2) + Math.pow(138 - (robot.pinpoint.getPosX(DistanceUnit.INCH) + robot.pinpoint.getVelX(DistanceUnit.INCH)/2), 2));
        double motorPower = robot.getshooterPower(distanceToGoal);
        double hoodAngle = robot.getHoodAngle(distanceToGoal);


        double shooterVelocity = robot.shooter.getVelocity();

        double output = shooterController.calculate(shooterVelocity, motorPower);
        robot.angleServo.setPosition(shooterAngle);
        if(gamepad1.a){
            robot.shooter.setVelocity(0);
        } else{
            robot.shooter.setVelocity(output);

        }

        telemetry.addData("x", robot.pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("y", robot.pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("x", robot.pinpoint.getVelX(DistanceUnit.INCH));
        telemetry.addData("y", robot.pinpoint.getVelY(DistanceUnit.INCH));
        telemetry.addData("anglewant", angleWant);
        telemetry.addData("target", target);
        telemetry.addData("facing", robotHeading);


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

        if (gamepad1.x) {
            if (robot.stopper.getPosition() == 0.47){
                robot.stopper.setPosition(0.7);
            } else {
                robot.stopper.setPosition(0.47);
            }
        }


        telemetry.update();

    }
    @Override
    public void stop(){robot.limelight.stop();}
    public double normA(double angle) {angle %= 360; if (angle < -134) angle += 360; else if (angle > 134) angle -= 360;return angle;}
    public double clamp(double x, double min, double max) {return Math.max(min,Math.min(max,x));}
}
