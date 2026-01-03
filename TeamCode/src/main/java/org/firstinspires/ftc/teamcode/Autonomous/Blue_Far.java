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

@Autonomous(name = "Blue far", group = "Examples")
public class Blue_Far extends OpMode {

    private HackinHoundsHardware robot = new HackinHoundsHardware();

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(57, 8, Math.toRadians(180));
    private final Pose scorePose = new Pose(53, 10, Math.toRadians(180));
    private final Pose pickupOne = new Pose(4,15,Math.toRadians(260));

    private final Pose pickupPointTwo = new Pose(6.77, 3, Math.toRadians(255));

    private final Pose curve1 = new Pose(37.8,20.7);

    private final Pose pickupTwo = new Pose (13.1,36.4, Math.toRadians(180));

    private final Pose curve2 = new Pose(66.3, 44);
    private final Pose pickupThree = new Pose (9,13 , Math.toRadians(180));

    private final Pose move = new Pose (34.6, 11.3, Math.toRadians(180));


    public double p = 0.025, i = 0, d = 0.0004, f = 0;

    public PIDFController turretController = new PIDFController(p, i, d, f);
    double turrettarget = 113;

    double target = 0;

    private FtcDashboard dashboard;

    public double P = 11, I = 0, D = 0, F = 0.8;
    public PIDFController shooterController = new PIDFController(P, I, D, F);
    double shooterTarget = 1480;
    double hoodAngle = 0.13;

    public double ticksPerDegree = 4.233;


    boolean wantToTrack = false;
    private Path scorepreload, pickup1, pickupPartTwo,  score1, pickup2, score2, pickup3, score3, park;








    public void buildPaths(){

        scorepreload = new Path(new BezierLine(startPose, scorePose));
        scorepreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        pickup1 = new Path(new BezierLine(scorePose, pickupOne));
        pickup1.setLinearHeadingInterpolation(scorePose.getHeading(), pickupOne.getHeading());

        pickupPartTwo = new Path(new BezierLine(pickupOne,pickupPointTwo));
        pickupPartTwo.setLinearHeadingInterpolation(pickupOne.getHeading(), pickupPointTwo.getHeading());

        score1 = new Path(new BezierCurve(pickupPointTwo, curve1, scorePose));
        score1.setLinearHeadingInterpolation(pickupPointTwo.getHeading(), scorePose.getHeading(), 0.9);

        pickup2 = new Path(new BezierCurve(scorePose, curve2, pickupTwo));
        pickup2.setLinearHeadingInterpolation(scorePose.getHeading(), pickupTwo.getHeading(), 0.7);

        score2 = new Path(new BezierLine(pickupTwo, scorePose));
        score2.setLinearHeadingInterpolation(pickupTwo.getHeading(), scorePose.getHeading(), 0.8);



        pickup3 = new Path(new BezierLine(scorePose, pickupThree));
        pickup3.setLinearHeadingInterpolation(scorePose.getHeading(), pickupThree.getHeading(), 0.8);


        score3 = new Path(new BezierLine(pickupThree, scorePose));
        score3.setLinearHeadingInterpolation(pickupThree.getHeading(), scorePose.getHeading(), 0.8);


        park = new Path(new BezierLine(scorePose, move));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), move.getHeading());



    }








    public void autonomousPathUpdate(){
        switch (pathState) {

            case 0:
                if (!follower.isBusy()) {
                    robot.stopper.setPosition(0.7);
                    follower.setMaxPower(0.7);
                    robot.intake.setPower(0.1);
                    follower.followPath(scorepreload);
                    wantToTrack = true;

                    setPathState(1);

                }
                break;

            case 1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 4) {
                    robot.intake.setPower(0.7);
                    robot.intake2.setPower(-0.7);
                    robot.stopper.setPosition(0.47);
                    wantToTrack = true;

                    setPathState(7);
                }
                break;


            case 7:
                if(pathTimer.getElapsedTimeSeconds() > 1){
                    follower.setMaxPower(0.5);
                    follower.followPath(pickup1, true);
                    robot.intake.setPower(0.7);
                    robot.intake2.setPower(-0.7);
                    robot.stopper.setPosition(0.67);
                    wantToTrack = false;
                    setPathState(8);


                }
                break;

            case 8:
                if(!follower.isBusy()){

                    robot.intake.setPower(0.7);
                    robot.intake2.setPower(-0.7);
                    follower.setMaxPower(0.4);
                    follower.followPath(pickupPartTwo, true);
                    setPathState(11);

                }
                break;





            case 11:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1){
                    robot.intake2.setPower(-0.7);
                    robot.intake.setPower(0.7);
                    follower.setMaxPower(1);
                    follower.followPath(score1, true);
                    wantToTrack = true;
                    setPathState(12);

                }
                break;


            case 12:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    robot.intake.setPower(0.7);
                    robot.intake2.setPower(-0.7);
                    robot.stopper.setPosition(0.47);
                    wantToTrack = true;

                    setPathState(13);
                }
                break;


            case 13:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1){
                    follower.setMaxPower(0.6);
                    follower.followPath(pickup2, true);
                   robot.intake.setPower(0.7);
                   robot.intake2.setPower(-0.7);
                   robot.stopper.setPosition(0.7);
                   wantToTrack = false;


                    setPathState(14);

                }
                break;


            case 14:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1){
                    robot.intake.setPower(0.7);
                    robot.intake2.setPower(-0.7);
                    robot.stopper.setPosition(0.7);
                    follower.followPath(score2);
                    wantToTrack = true;
                    setPathState(15);

                }
                break;

                case 15:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    robot.intake.setPower(0.7);
                    robot.intake2.setPower(-0.7);
                    robot.stopper.setPosition(0.47);
                    wantToTrack = true;

                    setPathState(16);
                }
                break;

            case 16:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    robot.intake.setPower(0.7);
                    robot.intake2.setPower(-0.7);
                    robot.stopper.setPosition(0.7);
                    follower.followPath(pickup3);
                    wantToTrack = true;

                    setPathState(17);
                }
                break;

            case 17:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    robot.intake.setPower(0.4);
                    robot.intake2.setPower(-0.4);
                    robot.stopper.setPosition(0.7);
                    follower.followPath(score3);
                    wantToTrack = true;

                    setPathState(18);
                }
                break;


            case 18:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    robot.intake.setPower(0.7);
                    robot.intake2.setPower(-0.7);
                    robot.stopper.setPosition(0.47);
                    wantToTrack = true;

                    setPathState(19);
                }
                break;


            case 19:
                if(pathTimer.getElapsedTimeSeconds() > 1.5){
                    robot.intake.setPower(0);
                    robot.intake2.setPower(0);
                    robot.stopper.setPosition(0.7);
                    wantToTrack = false;
                    follower.followPath(park);
                    turrettarget = 0;
                    shooterTarget = 0;
                    hoodAngle = 0;
                    setPathState(90);
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

        LLResult result = robot.limelight.getLatestResult();

        if(result.isValid() && wantToTrack){
            double target = normA(turrettarget - result.getTx() + 2.5);
            double turretPosition = robot.turret.getCurrentPosition()/4.233;
            if (target > 150) {target = 150;} else if (target < -150) {target = -150;}

            double output  = turretController.calculate(turretPosition, target);

            robot.turret.setPower(output);
        } else{
            double target = normA(turrettarget);
            if (target > 150) {target = 150;} else if (target < -150) {target = -150;}
            double turretPosition = robot.turret.getCurrentPosition()/4.233;
            double output  = turretController.calculate(turretPosition, target);

            robot.turret.setPower(output);
        }

        robot.angleServo.setPosition(hoodAngle);






            double shooterVelocity = robot.shooter.getVelocity();
            double shooterOutput = shooterController.calculate(shooterVelocity, shooterTarget);

//            robot.shooter.setPower(shooterOutput);

            robot.shooter.setVelocity(shooterOutput);


        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("shooterVelocity", robot.shooter.getVelocity());

        telemetry.update();



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

    public double normA(double angle) {angle %= 360; if (angle < -180) angle += 360; else if (angle > 180) angle -= 360;return angle;}

}
