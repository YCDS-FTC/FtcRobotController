package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.HackinHoundsHardware;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.RobotPose;

@Autonomous(name = "blue-close-visualizer", group = "Examples")
public class blue_close_visualizer extends OpMode {

    private HackinHoundsHardware robot = new HackinHoundsHardware();

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(17.6580310880829, 121.3678756476684, Math.toRadians(53));
    private final Pose scorePose = new Pose(52.651785714285715, 83.21428571428572, Math.toRadians(180));
    private final Pose pickupOne = new Pose(15,83, Math.toRadians(180));
    private final Pose gateEmpty = new Pose(15.46153846153846, 73, Math.toRadians(90));
    private final Pose goBack = new Pose(40.15384615384615, 78.46153846153847);
    private final Pose pickupTwo = new Pose (17,60, Math.toRadians(180));
    private final Pose curve1 = new Pose(80, 55);
    private final Pose pickupThree = new Pose(15.923076923076923, 35.07692307692308, Math.toRadians(180));
    private final Pose curve2 = new Pose(72.46153846153845, 28.61538461538461);
    private final Pose move = new Pose (22.615384615384613, 84.46153846153847, Math.toRadians(180));



    public double p = 0.02, i = 0, d = 0.0004, f = 0;

    public PIDFController turretController = new PIDFController(p, i, d, f);
    double Turrettarget = 0;



    public double P = 11, I = 0, D = 0, F = 0.8;
    public PIDFController shooterController = new PIDFController(P, I, D, F);
    double shooterTarget = 1130;

    public double ticksPerDegree = 4.233;

    private Path scorePreload, pickup1, emptyGate, score1, pickup2, score2, pickup3, score3, park;

    public Pose endAutoPose;

    boolean wantZero = false;



    //weird



    public void buildPaths(){
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading(), 0.8);


        pickup1 = new Path(new BezierLine(scorePose, pickupOne));
        pickup1.setLinearHeadingInterpolation(scorePose.getHeading(), pickupOne.getHeading(), 0.8);



        emptyGate = new Path(new BezierCurve(pickupOne, goBack, gateEmpty));
        emptyGate.setLinearHeadingInterpolation(pickupOne.getHeading(), gateEmpty.getHeading());


        score1 = new Path(new BezierLine(gateEmpty, scorePose));
        score1.setLinearHeadingInterpolation(pickupOne.getHeading(), scorePose.getHeading(), 0.9);

        pickup2 = new Path(new BezierCurve(scorePose, curve1, pickupTwo));
        pickup2.setLinearHeadingInterpolation(scorePose.getHeading(), pickupTwo.getHeading(), 0.7);

        score2 = new Path(new BezierLine(pickupTwo, scorePose));
        score2.setLinearHeadingInterpolation(pickupTwo.getHeading(), scorePose.getHeading(), 0.8);


        pickup3 = new Path(new BezierCurve(scorePose, curve2, pickupThree));
        pickup3.setLinearHeadingInterpolation(scorePose.getHeading(), pickupThree.getHeading(), 0.7);

        score3 = new Path(new BezierLine(pickupThree, scorePose));
        score3.setLinearHeadingInterpolation(pickupThree.getHeading(), scorePose.getHeading());

        park = new Path(new BezierLine(scorePose, move));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), move.getHeading());



    }








    public void autonomousPathUpdate(){
        switch (pathState) {

            case 0:
                if (!follower.isBusy()) {
                    robot.stopper.setPosition(0.7);
                    follower.followPath(scorePreload);
                    robot.intake.setPower(0.5);
                    robot.intake2.setPower(-0.3);

                    setPathState(1);

                }
                break;

            case 1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
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
                    robot.intake.setPower(1);
                    robot.intake2.setPower(-0.7);
                    robot.stopper.setPosition(0.7);
                    setPathState(20);

                }
                break;


            case 20:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {

                    follower.followPath(emptyGate);
                    robot.intake2.setPower(-0.3);
                    robot.intake.setPower(0.3);
                    setPathState(4);

                }
                break;



            case 4:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.5) {

                    follower.followPath(score1);
                    robot.intake2.setPower(-0.4);
                    robot.intake.setPower(0.4);
                    setPathState(5);

                }
                break;
            case 5:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3){
                    robot.intake.setPower(1);
                    robot.intake2.setPower(-0.85);
                    robot.stopper.setPosition(0.47);
                    setPathState(6);

                }
                break;

            case 6:
                if(pathTimer.getElapsedTimeSeconds() > 1){
                    follower.followPath(pickup2);
                    robot.intake.setPower(1);
                    robot.intake2.setPower(-0.7);
                    robot.stopper.setPosition(0.7);
                    setPathState(7);

                }
                break;

            case 7:
                if(!follower.isBusy()){
                    follower.followPath(score2);
                    robot.intake.setPower(0.3);
                    robot.intake2.setPower(-0.3);
                    setPathState(8);

                }
                break;

            case 8:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3){

                    robot.intake.setPower(1);
                    robot.intake2.setPower(-0.7);
                    robot.stopper.setPosition(0.47);
                    setPathState(9);

                }
                break;



            case 9:
                if(pathTimer.getElapsedTimeSeconds() > 1.8){
                    follower.followPath(pickup3);
                    follower.setMaxPower(0.6);
                    robot.stopper.setPosition(0.7);
                    robot.intake.setPower(1);
                    robot.intake2.setPower(-0.7);
                    setPathState(10);

                }
                break;


            case 10:
                if(!follower.isBusy()){
                    follower.followPath(score3);
                    follower.setMaxPower(1);
                    robot.intake.setPower(0.3);
                    robot.intake2.setPower(-0.3);
                    setPathState(11);

                }
                break;


            case 11:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3){
                    robot.intake2.setPower(-0.7);
                    robot.intake.setPower(0.7);
                    robot.stopper.setPosition(0.47);
                    setPathState(12);

                }
                break;


            case 12:
                if(pathTimer.getElapsedTimeSeconds() > 1){
                    robot.stopper.setPosition(0.7);
                    robot.intake.setPower(0);
                    robot.intake2.setPower(0);
                    Turrettarget = 0;
                    shooterTarget = 0;
                    wantZero = true;
                    follower.followPath(park);

                    setPathState(1000);

                }
                break;


            case 13:
                if(!follower.isBusy()){
                    robot.intake.setPower(0.7);
                    robot.intake2.setPower(-0.7);
                    robot.stopper.setPosition(0.47);
                    setPathState(14);

                }
                break;


            case 14:
                if(!follower.isBusy()){
                    robot.intake.setPower(0.7);
                    robot.intake2.setPower(-0.7);
                    robot.stopper.setPosition(0.47);
                    setPathState(15);

                }
                break;

            case 15:
                if(!follower.isBusy()){
                    robot.intake.setPower(0.7);
                    robot.intake2.setPower(-0.7);
                    robot.stopper.setPosition(0.47);


                }
                break;

//
        }

    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop(){
        LLResult result = robot.limelight.getLatestResult();

        double ty = result.getTy();
        double tx = result.getTx();

        double distanceToGoal =  robot.limelight(ty, tx);
        double motorPower = robot.getshooterPower(distanceToGoal);
        double hoodAngle = robot.getHoodAngle(distanceToGoal);

        double shooterVelocity = robot.shooter.getVelocity();

        double output = shooterController.calculate(shooterVelocity, shooterTarget);

//        robot.shooter.setVelocity(output);



        double robotHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - 53;
        //double robotHeading = robot.pinpoint.getHeading(AngleUnit.DEGREES);
        //if (gamepad1.right_trigger > 0.1) {angleWant = robotHeading;}
        double turretAngle = robot.turret.getCurrentPosition()/ticksPerDegree;

            //Turrettarget = (robotHeading + turretAngle);
        Turrettarget = Math.toDegrees(Math.atan2(140 - robot.pinpoint.getPosY(DistanceUnit.INCH), 3 + robot.pinpoint.getPosX(DistanceUnit.INCH))) - 180;


        if (result.isValid() && !gamepad1.left_bumper) {
            Turrettarget = (robotHeading + turretAngle) - tx;
        }


        if (wantZero) {Turrettarget = 0;}

        double target = normA(Turrettarget - robotHeading);
        if (target > 150) {target = 150;} else if (target < -150) {target = -150;}
//        double error = target - turretAngle;
//        double turretPower = clamp(error * slow, -1, 1);
//        robot.turret.setPower(turretController.calculate(turretAngle, target));





        RobotPose.endPose = endAutoPose;

        endAutoPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());




        follower.update();
        autonomousPathUpdate();







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



    }
    @Override
    public void init(){
        robot.init(hardwareMap);

        turretController.setPIDF(p,i,d,f);


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
