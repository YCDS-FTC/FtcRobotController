package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.HackinHoundsHardware;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;

public class FAR_BLUE extends OpMode {

    private HackinHoundsHardware robot = new HackinHoundsHardware();

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(41.54, 11.75, Math.toRadians(180));
    private final Pose scorePose = new Pose(38.99, 12.47, Math.toRadians(180));
    private final Pose pickupOne = new Pose(-8,21.31, Math.toRadians(260.6958));
    private final Pose pickingOne = new Pose (-10.50,7.84, Math.toRadians(85.9));
    private final Pose curve1 = new Pose(95, 55);
    private final Pose pickupThree = new Pose(125, 35, Math.toRadians(0));
    private final Pose curve2 = new Pose(79, 36);
    private final Pose move = new Pose (84, 60, Math.toRadians(0));

    public double p = 0.02, i = 0, d = 0.0004, f = 0;

    public PIDFController turretController = new PIDFController(p, i, d, f);
    double Turrettarget = 0;



    public double P = 11, I = 0, D = 0, F = 0.8;
    public PIDFController shooterController = new PIDFController(P, I, D, F);
    double shooterTarget = 1480;

    double hoodTarget = .13;

    public double ticksPerDegree = 4.233;

    private Path scorePreload, pickup1, score1, actualPick, pickup2, score2, pickup3, score3, park;

    boolean wantZero = false;



    public void buildPaths(){
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading(), 0.8);


        pickup1 = new Path(new BezierLine(scorePose, pickupOne));
        pickup1.setLinearHeadingInterpolation(scorePose.getHeading(), pickupOne.getHeading(), 0.7);

        actualPick = new Path(new BezierLine(pickupOne,pickingOne));
        actualPick.setLinearHeadingInterpolation(pickupOne.getHeading(), pickingOne.getHeading());

        score1 = new Path(new BezierLine(pickupOne, scorePose));
        score1.setLinearHeadingInterpolation(pickupOne.getHeading(), scorePose.getHeading(), 0.9);

//        pickup2 = new Path(new BezierCurve(scorePose, curve1, pickupTwo));
//        pickup2.setLinearHeadingInterpolation(scorePose.getHeading(), pickupTwo.getHeading(), 0.7);
//
//        score2 = new Path(new BezierLine(pickupTwo, scorePose));
//        score2.setLinearHeadingInterpolation(pickupTwo.getHeading(), scorePose.getHeading(), 0.8);


        pickup3 = new Path(new BezierCurve(scorePose, curve2, pickupThree));
        pickup3.setLinearHeadingInterpolation(scorePose.getHeading(), pickupThree.getHeading(), 0.7);

        score3 = new Path(new BezierLine(pickupThree, scorePose));
        score3.setLinearHeadingInterpolation(pickupThree.getHeading(), scorePose.getHeading());

        park = new Path(new BezierLine(scorePose, move));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), move.getHeading());



    }






    public void autonomousPathUpdate(){
        switch (pathState){

            case 0:
                if (!follower.isBusy()) {
                    robot.stopper.setPosition(0.7);
                    robot.intake.setPower(0.5);
                    robot.intake2.setPower(-0.3);

                    setPathState(1);

                }
                break;

            case 1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 6) {
                    robot.intake.setPower(1);
                    robot.intake2.setPower(-.7);
                    robot.stopper.setPosition(0.47);
                    setPathState(3);
                }
                break;



            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 1) {

                    follower.followPath(pickup1);
                    follower.setMaxPower(0.6);
                    robot.intake.setPower(.4);
                    robot.intake2.setPower(-0.4);
                    robot.stopper.setPosition(0.7);
                    setPathState(20);

                }
                break;


            case 20:
                if (!follower.isBusy()) {

                    follower.followPath(actualPick);
                    robot.intake2.setPower(-0.7);
                    robot.intake.setPower(0.7);
                    setPathState(4);

                }
                break;
            case 4:
                if (!follower.isBusy()){
                    follower.followPath(score1);
                    setPathState(10);
                }
                break;

            case 10:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 4){

                    robot.intake.setPower(1);
                    robot.intake2.setPower(-0.7);
                    robot.stopper.setPosition(0.47);
                    setPathState(9);

                }
                break;
        }

    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        LLResult result = robot.limelight.getLatestResult();

        double ty = result.getTy();
        double tx = result.getTx();



        double shooterVelocity = robot.shooter.getVelocity();

        double output = shooterController.calculate(shooterVelocity, shooterTarget);

        robot.shooter.setVelocity(output);


        double robotHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - 180;
        //double robotHeading = robot.pinpoint.getHeading(AngleUnit.DEGREES);
        //if (gamepad1.right_trigger > 0.1) {angleWant = robotHeading;}
        double turretAngle = robot.turret.getCurrentPosition() / ticksPerDegree;

        //Turrettarget = (robotHeading + turretAngle);
        Turrettarget = Math.toDegrees(Math.atan2(140 - robot.pinpoint.getPosY(DistanceUnit.INCH), -robot.pinpoint.getPosX(DistanceUnit.INCH))) - 180;


        if (result.isValid() && !gamepad1.left_bumper) {
            Turrettarget = (robotHeading + turretAngle) - tx + 2;
        }
        if (wantZero) {
            Turrettarget = 75;
        }

        double target = normA(Turrettarget - robotHeading);
        if (target > 150) {
            target = 150;
        } else if (target < -150) {
            target = -150;
        }
//        double error = target - turretAngle;
//        double turretPower = clamp(error * slow, -1, 1);
        robot.turret.setPower(turretController.calculate(turretAngle, target));


        robot.angleServo.setPosition(hoodTarget);
        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("imu", "%f", robotHeading);

        telemetry.addData("turretPos", "%d", robot.turret.getCurrentPosition());
        telemetry.addData("turretAngle", "%f", turretAngle);
        telemetry.addData("turretTarget", "%f", target);
        telemetry.addData("target", "%f", Turrettarget);
        telemetry.addData("turretPower", "%f", robot.turret.getVelocity());
        telemetry.addData("Tx", "%f", tx);
        telemetry.update();

        result = robot.limelight.getLatestResult();



    }
    @Override
    public void init(){
        robot.init(hardwareMap);



        robot.limelight.pipelineSwitch(0);



        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);


    }



    @Override
    public void init_loop(){

    }

    @Override
    public void start(){
        robot.limelight.start();
        opmodeTimer.resetTimer();
        setPathState(0);

    }




    @Override
    public void stop(){

    }

    public double normA(double angle) {angle %= 360; if (angle < -180) angle += 360; else if (angle > 180) angle -= 360;return angle;}

}
