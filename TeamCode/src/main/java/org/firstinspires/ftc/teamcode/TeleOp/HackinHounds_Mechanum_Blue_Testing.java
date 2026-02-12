package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.PedroPathing.Tuning.follower;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.HackinHoundsHardware;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.RobotPose;

@Config
@TeleOp(name = "Mechanum-Blue-test", group = "Linear OpMode")
public class HackinHounds_Mechanum_Blue_Testing extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();
    private HackinHoundsHardware robot = new HackinHoundsHardware();

    private FtcDashboard dashboard;

    double shift = 1;

    public  double transferPower = 0;
    public  double intakePower = 0;

    private double filteredVar = 0;
    private boolean filterStart = false;
    private double filterTick = 0;


    private  double turret_tPERd = 4.233;
    private  double angleWant = 125;
    private  double slow = 1;

    public static double p = 0.03;
    public static double i = 0;
    public static double d = 0.0004;
    public static double f = 0.0001;
    public static double ks = 50;

    PIDFController turretController = new PIDFController(p,i,d,f);


    public static double kp = 11;
    public static double ki = 0;
    public static double kd = 2;
    public static double kf = 1;


    PIDFController shooterController = new PIDFController(kp, ki, kd, kf);

    public static double shooterAngle = 0;
    public static double stopperPosition = .47;
    //position is 0.44 for stopping and 0.63 for neutral

    public static double shootertarget = 0;

    public static double target = 0;

    boolean isBlockerClosed = true;

    boolean isWorking = true;
    boolean rightBumper_pressed_previous = false;

    boolean leftBumper_pressed_previous = false;

    public static double xPrediction = 0.8;
    public static double yPrediction = 0.8;


    public double xOffset = 0;
    public double yOffset = 0;

    double robotHeadingOffset = RobotPose.endHeading;

    public double goalX = 0.5;
    public double goalY = 144;

    Pose startPose = RobotPose.endPose;

    private ElapsedTime stopperTimer = new ElapsedTime();
    private boolean isStopperTimedOpen = false;
    private boolean isSingleStopperTimedOpen = false;

    private double xVelocity;
    private double yVelocity;
    private double ballMovementClose = 5;


    private double distanceToGoal;

    @Override
    public void init(){
        robot.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);


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


        follower.update();


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

        if (gamepad1.right_trigger > 0.3) {
            HuskyLens.Block[] blocks = robot.huskyLens.blocks();
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
                double assistPower = dis * 0.5;
                double angle = facing + (Math.PI / 2.0);
                y += assistPower * Math.cos(angle);
                x -= assistPower * Math.sin(angle);
            }
        }

        double rotX = x * Math.cos(-facing) - y * Math.sin(-facing);
        rotX = rotX * 1.1;
        double rotY = x * Math.sin(-facing) + y * Math.cos(-facing);

        double d = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(rx), 1);

        double lf = (rotY + rotX + rx) / d;
        double lb = (rotY - rotX + rx) / d;
        double rf = (rotY - rotX - rx) / d;
        double rb = (rotY + rotX - rx) / d;

        robot.leftBack.setPower(lb * shift);
        robot.leftFront.setPower(lf * shift);
        robot.rightBack.setPower(rb * shift);
        robot.rightFront.setPower(rf * shift);
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
            robot.intake2.setPower(-0.5);
            robot.intake.setPower(0.5);
        }

        if(gamepad2.right_trigger > Math.abs(0.1)) {
            robot.intake.setPower(-0.7);
        }



        if (gamepad1.left_bumper) {
            angleWant = 118;
        }

        if (gamepad1.left_trigger > 0.4){
            robot.shooter.setVelocity(1260);
            robot.angleServo.setPosition(0.13);

        }

        //Stopper logic for ONE BY ONE
        if(gamepad2.left_bumper && !leftBumper_pressed_previous) {
            if (isBlockerClosed) {
                robot.stopper.setPosition(0.47);
                isBlockerClosed = false;
                isSingleStopperTimedOpen = true;
                robot.intake.setPower(1);
                robot.intake2.setPower(-1);
                timer.reset();
            } else {
                robot.stopper.setPosition(0.67);
                isBlockerClosed = true;
                isSingleStopperTimedOpen = false;
                robot.intake.setPower(0.7);
                robot.intake2.setPower(-0.7);


            }
        }

            if(isSingleStopperTimedOpen && timer.seconds() > 1.5){
                robot.stopper.setPosition(0.7);
                isBlockerClosed = true;
                isSingleStopperTimedOpen = false;
                robot.intake.setPower(0.5);
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



//        double distanceToGoal =  robot.limelight(robotX,robotY);


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



        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();


//        distanceToGoal = robot.odometryDistance(robotX, robotY);


        if (gamepad1.b){
            follower.setPose(new Pose(135, 7.625, Math.toRadians(180)));
        }


//        robot.shooter.setVelocity(motorPower);

        double robXV = robot.pinpoint.getVelX(DistanceUnit.INCH) * xPrediction;
        double robYV = robot.pinpoint.getVelY(DistanceUnit.INCH) * yPrediction;

        if(gamepad2.dpad_right){
            goalX += 2;
        }

        if(gamepad2.dpad_left){
            goalX -= 2;
        }

        double dx = goalX - (robotX + robXV);
        double dy = goalY - (robotY + robYV);

        double predictedX = robotX + robXV;
        double predictedY = robotY + robYV;


        distanceToGoal = robot.odometryDistanceBlue(predictedX, predictedY);


        double goalHeadingField = Math.atan2(-dy, -dx);
        double goalHeadingFieldDegrees = Math.toDegrees(goalHeadingField);

        double robotHeading = follower.getPose().getHeading();
        double robotHeadingDegrees = Math.toDegrees(robotHeading);

        double turretTargetAngle = goalHeadingFieldDegrees - robotHeadingDegrees;
        double turretAngle = robot.turret.getCurrentPosition()/turret_tPERd;


        double target = normA(turretTargetAngle);

        if (target > 150) {target = 150;} else if (target < -150) {target = -150;}
//        double error = target - turretAngle;
//        double turretPower = clamp(error * slow, -1, 1);
        double turretPower = (turretController.calculate(turretAngle, target));


        robot.turret.setPower(turretPower);



        double motorPower = robot.getshooterPower(distanceToGoal);
        double hoodAngle = robot.getHoodAngle(distanceToGoal);




        double shooterVelocity = robot.shooter.getVelocity();

        double output = shooterController.calculate(shooterVelocity, motorPower);



        if(gamepad1.a){
            robot.shooter.setVelocity(0);
        } else if (gamepad1.left_trigger <= 0.4){
            robot.angleServo.setPosition(hoodAngle);
            robot.shooter.setVelocity(output);

        }

        robot.lights(robot.light1, robot.light2, robot.light3, robot.color0, robot.color1, robot.color2, robot.color3);

        if ( Math.abs(robot.angleServo.getPosition() - hoodAngle)  < 0.01 && Math.abs(robot.shooter.getVelocity() - motorPower) < 50  && turretTargetAngle - turretAngle < 2){
            robot.light4.setPosition(.611);
        } else{
            robot.light4.setPosition(0);
        }




        telemetry.addData("robotHeadingRadians", "%f", robotHeading);
        telemetry.addData("robotHeadingDegrees", "%f", robotHeadingDegrees);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("dx", "%f", dx);
        telemetry.addData("dy","%f", dy);
        telemetry.addData("Goalheadingfield","%f", goalHeadingFieldDegrees);
        telemetry.addData("target","%f", target);
        telemetry.addData("turretPos", "%f", turretAngle);
        telemetry.addData("distance", distanceToGoal);

        telemetry.addData("goalX", goalX);
        telemetry.addData("tx", tx);
//        telemetry.addData("front:", "%f", robot.intake.getPower());
//        telemetry.addData("back:", "%f", robot.intake2.getPower());
//        telemetry.addData("turretPos", "%f", robot.turret.getCurrentPosition()/4.233);
//        telemetry.addData("turretTarget", "%f", target);
//        telemetry.addData("error", "%f", error);
        telemetry.addData("turretPower", "%f", robot.turret.getVelocity());
        telemetry.addData("distancetogoal", distanceToGoal);
//        telemetry.addData("targetShootPower", motorPower);

        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("shooterVelocity", robot.shooter.getVelocity());
        packet.put("shooterRPM", (robot.shooter.getVelocity() / 28.0) * 60.0);
        packet.put("shootertarget", shootertarget);
        packet.put("turretTarget",  target);

        dashboard.sendTelemetryPacket(packet);

    }







    public double normA(double angle) {angle %= 360; if (angle < -180) angle += 360; else if (angle > 180) angle -= 360;return angle;}
    public double clamp(double x, double min, double max) {return Math.max(min,Math.min(max,x));}

}
