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

    private final Pose startPose = new Pose(55.95854922279792, 9, Math.toRadians(90));
    private final Pose scorePose = new Pose(52.9740932642487, 15, Math.toRadians(180));
    private final Pose pickupOne = new Pose(0,19, Math.toRadians(250));
    private final Pose pickupTwo = new Pose (7,44, Math.toRadians(180));
    private final Pose pickupPointTwo = new Pose(-5, 9, Math.toRadians(250));
    private final Pose pickupPointThree = new Pose(13, 12, Math.toRadians(185));
    private final Pose pickupPointFour = new Pose(2, 12, Math.toRadians(185));

    private final Pose curve2 = new Pose(67.39896373056995, 41.284974093264246);
    private final Pose move = new Pose (52.2279792746114, 36.55958549222798, Math.toRadians(180));


    public double p = 0.025, i = 0, d = 0.0004, f = 0;

    public PIDFController turretController = new PIDFController(p, i, d, f);
    double target = 114;


    private FtcDashboard dashboard;

    public double P = 11, I = 0, D = 0, F = 0.8;
    public PIDFController shooterController = new PIDFController(P, I, D, F);
    double shooterTarget = 1550;
    double hoodAngle = 0.2;

    public double ticksPerDegree = 4.233;

    private Path scorePreload, pickup1, pickupPartTwo, pickupPartThree, pickupPartFour, score1, pickup2, score2, pickup3, score3, park;








    public void buildPaths(){
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading(), 0.8);



        pickup1 = new Path(new BezierLine(scorePose, pickupOne));
        pickup1.setLinearHeadingInterpolation(scorePose.getHeading(), pickupOne.getHeading());

        pickupPartTwo = new Path(new BezierLine(pickupOne,pickupPointTwo));
        pickupPartTwo.setLinearHeadingInterpolation(pickupOne.getHeading(), pickupPointTwo.getHeading());

        pickupPartThree = new Path(new BezierLine(pickupPointTwo, pickupPointThree));
        pickupPartThree.setLinearHeadingInterpolation(pickupPointTwo.getHeading(), pickupPointThree.getHeading());

        pickupPartFour = new Path(new BezierLine(pickupPointThree, pickupPointFour));
        pickupPartFour.setLinearHeadingInterpolation(pickupPointThree.getHeading(), pickupPointFour.getHeading());

        score1 = new Path(new BezierLine(pickupPointFour, scorePose));
        score1.setLinearHeadingInterpolation(pickupPointFour.getHeading(), scorePose.getHeading(), 0.9);

        pickup2 = new Path(new BezierCurve(scorePose, curve2, pickupTwo));
        pickup2.setLinearHeadingInterpolation(scorePose.getHeading(), pickupTwo.getHeading(), 0.7);

        score2 = new Path(new BezierLine(pickupTwo, scorePose));
        score2.setLinearHeadingInterpolation(pickupTwo.getHeading(), scorePose.getHeading(), 0.8);

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
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.5) {
                    robot.intake.setPower(0.7);
                    robot.intake2.setPower(-0.7);
                    setPathState(2);
                }
                break;


            case 2:
                if(pathTimer.getElapsedTimeSeconds() > 1){
                robot.intake.setPower(0.7);
                robot.intake2.setPower(-0.7);
                robot.stopper.setPosition(0.47);


//                    if(robot.flick.getCurrentPosition() > 49){
//                        robot.flick.setTargetPosition(0);
//                    }

                setPathState(3);
            }

            case 3:
                if (pathTimer.getElapsedTimeSeconds() > .3) {
                    robot.intake.setPower(0.7);
                    robot.intake2.setPower(-0.7);
                    robot.stopper.setPosition(0.67);
                    setPathState(4);

                }
                break;

            case 4:
                if(pathTimer.getElapsedTimeSeconds() > 1){
                    robot.intake.setPower(0.7);
                    robot.intake2.setPower(-0.7);
                    robot.stopper.setPosition(0.47);
                    setPathState(5);

                }
                break;
            case 5:
                if(pathTimer.getElapsedTimeSeconds() > 0.3){
                    robot.intake.setPower(0.7);
                    robot.intake2.setPower(-0.7);
                    robot.stopper.setPosition(0.67);
                    setPathState(6);

                }
                break;

            case 6:
                if(pathTimer.getElapsedTimeSeconds() > 1){
                    robot.intake.setPower(0.7);
                    robot.intake2.setPower(-0.7);
                    robot.stopper.setPosition(0.47);
                    setPathState(7);

                }
                break;

            case 7:
                if(pathTimer.getElapsedTimeSeconds() > 0.5){
                    follower.setMaxPower(1);
                    follower.followPath(pickup1);
                    robot.intake.setPower(0.7);
                    robot.intake2.setPower(-0.7);
                    robot.stopper.setPosition(0.67);
                    setPathState(8);

                }
                break;

            case 8:
                if(!follower.isBusy()){

                    robot.intake.setPower(0.7);
                    robot.intake2.setPower(-0.7);
                    follower.setMaxPower(0.3);
                    follower.followPath(pickupPartTwo);
                    setPathState(9);

                }
                break;



            case 9:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2){
                    follower.setMaxPower(0.3);
                    follower.followPath(pickupPartThree);
                    robot.stopper.setPosition(0.67);
                    robot.intake.setPower(0.7);
                    robot.intake2.setPower(-0.7);
                    setPathState(10);

                }
                break;


            case 10:
                if(!follower.isBusy()){
                    follower.setMaxPower(1);
                    follower.followPath(pickupPartFour);

                    robot.intake.setPower(0.3);
                    robot.intake2.setPower(-0.3);
                    setPathState(11);

                }
                break;


            case 11:
                if(!follower.isBusy()){
                    robot.intake2.setPower(-0.7);
                    robot.intake.setPower(0.7);
                    follower.followPath(score1);
                    follower.setMaxPower(1);
                    setPathState(12);

                }
                break;


            case 12:
                if(!follower.isBusy()){
                   follower.followPath(pickup2);
                   follower.setMaxPower(0.4);
                   robot.intake.setPower(0.7);
                   robot.intake2.setPower(-0.7);
                   robot.stopper.setPosition(0.67);


                    setPathState(13);

                }
                break;


            case 13:
                if(!follower.isBusy()){
                    robot.intake.setPower(0.7);
                    robot.intake2.setPower(-0.7);
                    robot.stopper.setPosition(0.67);
                    follower.followPath(score2);
                    setPathState(14);

                }
                break;


            case 14:
            if(!follower.isBusy()){
                robot.intake.setPower(0.7);
                robot.intake2.setPower(-0.7);
                robot.stopper.setPosition(0.47);


//                    if(robot.flick.getCurrentPosition() > 49){
                setPathState(15);
            }
            break;

            case 15:
                if (pathTimer.getElapsedTimeSeconds() > .3) {
                    robot.intake.setPower(0.3);
                    robot.intake2.setPower(-0.3);
                    robot.stopper.setPosition(0.67);
                    setPathState(16);

                }
                break;

            case 16:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    robot.intake.setPower(0.7);
                    robot.intake2.setPower(-0.7);
                    robot.stopper.setPosition(.47);
                    setPathState(17);

                }
                break;
            case 17:
                if(pathTimer.getElapsedTimeSeconds() > 0.3){
                    robot.intake.setPower(0.3);
                    robot.intake2.setPower(-0.3);
                    robot.stopper.setPosition(0.67);
                    setPathState(18);

                }
                break;

            case 18:
                if(pathTimer.getElapsedTimeSeconds() > 0.5){
                    robot.intake.setPower(0.7);
                    robot.intake2.setPower(-0.7);
                    robot.stopper.setPosition(0.47);
                    setPathState(19);

                }
                break;


            case 19:
                if(pathTimer.getElapsedTimeSeconds() > 1.5){
                    robot.intake.setPower(0);
                    robot.intake2.setPower(0);
                    robot.stopper.setPosition(0.67);
                    follower.followPath(park);
                    target = 0;
                    shooterTarget = 0;
                    setPathState(15);
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

            robot.angleServo.setPosition(hoodAngle);



            double shooterVelocity = robot.shooter.getVelocity();
            double shooterOutput = shooterController.calculate(shooterVelocity, shooterTarget);

//            robot.shooter.setPower(shooterOutput);

            robot.shooter.setVelocity(shooterTarget);


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
