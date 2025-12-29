package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
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

import org.firstinspires.ftc.teamcode.Hardware.HackinHoundsHardware;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;

public class Blue_Close_No_Gate_Old extends OpMode {

    private HackinHoundsHardware robot = new HackinHoundsHardware();

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(17.6580310880829, 121.3678756476684, Math.toRadians(55));
    private final Pose scorePose = new Pose(45, 90, Math.toRadians(180));
    private final Pose pickupOne = new Pose(7,92, Math.toRadians(180));
    private final Pose gateEmpty = new Pose(14, 81, Math.toRadians(90));
    private final Pose goback = new Pose(24, 70, Math.toRadians(180));
    private final Pose pickupTwo = new Pose (17,70, Math.toRadians(180));
    private final Pose curve1 = new Pose(62, 55);
    private final Pose pickupThree = new Pose(15, 48, Math.toRadians(180));
    private final Pose curve2 = new Pose(60, 25);
    private final Pose move = new Pose (28, 72.3, Math.toRadians(180));


    public double p = 0.025, i = 0, d = 0.0004, f = 0;

    public PIDFController turretController = new PIDFController(p, i, d, f);
    double target = 133;


    private FtcDashboard dashboard;

    public double P = 11, I = 0, D = 0, F = 0.8;
    public PIDFController shooterController = new PIDFController(P, I, D, F);
    double shooterTarget = 1200;

    public double ticksPerDegree = 4.233;

    private Path scorePreload, pickup1, goBack, emptygate, score1, pickup2, score2, pickup3, score3, park;








    public void buildPaths(){
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading(), 0.8);


        pickup1 = new Path(new BezierLine(scorePose, pickupOne));
        pickup1.setLinearHeadingInterpolation(scorePose.getHeading(), pickupOne.getHeading(), 0.7);

        goBack = new Path(new BezierLine(pickupTwo, goback));
        goBack.setConstantHeadingInterpolation(goback.getHeading());

        emptygate = new Path(new BezierLine(goback, gateEmpty));
        emptygate.setLinearHeadingInterpolation(goback.getHeading(), gateEmpty.getHeading());


        score1 = new Path(new BezierLine(pickupOne, scorePose));
        score1.setLinearHeadingInterpolation(gateEmpty.getHeading(), scorePose.getHeading(), 0.9);

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
                    robot.stopper.setPosition(0.67);
                    follower.setMaxPower(0.7);
                    follower.followPath(scorePreload);
                    robot.intake.setPower(0.5);
                    robot.intake2.setPower(-0.3);

                    setPathState(1);

                }
                break;

            case 1:
                if (!follower.isBusy()) {
                    robot.intake.setPower(0.7);
                    robot.intake2.setPower(-0.7);
                    setPathState(2);
                }
                break;


            case 2: {
                robot.stopper.setPosition(0.47);


//                    if(robot.flick.getCurrentPosition() > 49){
//                        robot.flick.setTargetPosition(0);
//                    }

                setPathState(3);
            }

            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {

                    follower.setMaxPower(0.5);
                    follower.followPath(pickup1);
                    robot.intake.setPower(1);
                    robot.intake2.setPower(-0.7);
                    robot.stopper.setPosition(0.67);
                    setPathState(6);

                }
                break;

//            case 4:
//                if(!follower.isBusy()){
//                    follower.followPath(goBack);
//                    robot.intake2.setPower(-0.3);
//                    robot.intake.setPower(0.3);
//                    setPathState(5);
//                }
//                break;
//
//
//            case 5:
//                if(!follower.isBusy()){
//                    follower.followPath(emptygate);
//                    robot.intake2.setPower(-0.3);
//                    robot.intake.setPower(0.3);
//                    setPathState(6);
//                }
//                break;


            case 6:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {

                    follower.setMaxPower(1);
                    follower.followPath(score1);
                    robot.intake2.setPower(-0.3);
                    robot.intake.setPower(0.3);
                    setPathState(7);

                }
                break;
            case 7:
                if(!follower.isBusy()){
                    robot.intake.setPower(0.7);
                    robot.intake2.setPower(-0.7);
                    robot.stopper.setPosition(0.47);
                    setPathState(8);

                }
                break;

            case 8:
                if(pathTimer.getElapsedTimeSeconds() > 2){
                    follower.setMaxPower(0.4);
                    follower.followPath(pickup2);
                    robot.intake.setPower(1);
                    robot.intake2.setPower(-0.7);
                    robot.stopper.setPosition(0.67);
                    setPathState(9);

                }
                break;

            case 9:
                if(!follower.isBusy()){
                    follower.setMaxPower(1);
                    follower.followPath(score2);
                    robot.intake.setPower(0.3);
                    robot.intake2.setPower(-0.3);
                    setPathState(10);

                }
                break;

            case 10:
                if(!follower.isBusy()){

                    robot.intake.setPower(0.7);
                    robot.intake2.setPower(-0.7);
                    robot.stopper.setPosition(0.47);
                    setPathState(11);

                }
                break;



            case 11:
                if(pathTimer.getElapsedTimeSeconds() > 1.5){
                    follower.setMaxPower(0.3);
                    follower.followPath(pickup3);
                    robot.stopper.setPosition(0.67);
                    robot.intake.setPower(1);
                    robot.intake2.setPower(-0.7);
                    setPathState(12);

                }
                break;


            case 12:
                if(!follower.isBusy()){
                    follower.setMaxPower(1);
                    follower.followPath(score3);

                    robot.intake.setPower(0.3);
                    robot.intake2.setPower(-0.3);
                    setPathState(13);

                }
                break;


            case 13:
                if(!follower.isBusy()){
                    robot.intake2.setPower(-0.7);
                    robot.intake.setPower(0.7);
                    robot.stopper.setPosition(0.47);
                    setPathState(14);

                }
                break;


            case 14:
                if(pathTimer.getElapsedTimeSeconds() > 2.4){
                    robot.stopper.setPosition(0.67);
                    robot.intake.setPower(0);
                    robot.intake2.setPower(0);
                    target = 0;
                    shooterTarget = 0;
                    follower.followPath(park);

                    setPathState(15);

                }
                break;


            case 15:
                if(!follower.isBusy()){
                    robot.intake.setPower(0.7);
                    robot.intake2.setPower(-0.7);
                    robot.stopper.setPosition(0.47);
                    setPathState(16);

                }
                break;


            case 16:
                if(!follower.isBusy()){
                    robot.intake.setPower(0.7);
                    robot.intake2.setPower(-0.7);
                    robot.stopper.setPosition(0.47);
                    setPathState(17);

                }
                break;

            case 17:
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
        follower.update();
        autonomousPathUpdate();



            double turretPosition = robot.turret.getCurrentPosition()/4.233;
            double output  = turretController.calculate(turretPosition, target);

            robot.turret.setPower(output);




            double shooterVelocity = robot.shooter.getVelocity();
            double shooterOutput = shooterController.calculate(shooterVelocity, shooterTarget);

//            robot.shooter.setPower(shooterOutput);

            robot.shooter.setVelocity(shooterOutput);


        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());

        telemetry.update();


        LLResult result = robot.limelight.getLatestResult();

        double ty = result.getTy();
        double tx = result.getTx();

        double distanceToGoal =  robot.limelight(ty, tx);
        double motorPower = robot.getshooterPower(distanceToGoal);
        double hoodAngle = robot.getHoodAngle(distanceToGoal);


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
}
