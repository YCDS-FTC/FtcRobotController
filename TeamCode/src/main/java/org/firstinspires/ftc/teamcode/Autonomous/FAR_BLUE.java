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

import org.firstinspires.ftc.teamcode.Hardware.HackinHoundsHardware;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;

@Autonomous(name = "DING DONG auto far blue", group = "Examples")
public class FAR_BLUE extends OpMode {

    private HackinHoundsHardware robot = new HackinHoundsHardware();

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(60, 9, Math.toRadians(90));
    private final Pose scorePose = new Pose(60, 16, Math.toRadians(135));
    private final Pose pickupOne = new Pose(125,83, Math.toRadians(0));
    private final Pose pickupTwo = new Pose (126,60, Math.toRadians(0));
    private final Pose curve1 = new Pose(95, 55);
    private final Pose pickupThree = new Pose(125, 35, Math.toRadians(0));
    private final Pose curve2 = new Pose(79, 36);
    private final Pose move = new Pose (84, 60, Math.toRadians(0));



    private Path scorePreload, pickup1, score1, pickup2, score2, pickup3, score3, park;



    public void buildPaths(){
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading(), 0.8);


        pickup1 = new Path(new BezierLine(scorePose, pickupOne));
        pickup1.setLinearHeadingInterpolation(scorePose.getHeading(), pickupOne.getHeading(), 0.7);

        score1 = new Path(new BezierLine(pickupOne, scorePose));
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
        switch (pathState){

            case 0:
                if(!follower.isBusy()){

                    follower.setMaxPower(0.7);
                    follower.followPath(scorePreload);

                    setPathState(1);

                }
                break;

            case 1:
                if (pathTimer.getElapsedTimeSeconds() > 3.0){

                    LLResult result = robot.limelight.getLatestResult();

                    double ty = result.getTy();
                    double tx = result.getTx();

                    double distanceToGoal =  robot.limelight(ty, tx);
                    setPathState(2);
                }
                break;


            case 2:
                if(pathTimer.getElapsedTimeSeconds() > 5){

                    robot.flick.setTargetPosition(50);
                    robot.flick.setPower(1);

//                    if(robot.flick.getCurrentPosition() > 49){
//                        robot.flick.setTargetPosition(0);
//                    }

                    setPathState(3);
                }
                if(pathTimer.getElapsedTimeSeconds() > 6) {
                    robot.flick.setTargetPosition(0);
                    robot.flick.setPower(1);
                }
                break;

            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 8){

                    robot.intake.setPosition(0);
                    robot.intake2.setPosition(1);
                    setPathState(4);

                }
                break;

            case 4:
                if(pathTimer.getElapsedTimeSeconds() > 10){



                    robot.flick.setTargetPosition(50);
                    robot.flick.setPower(1);

                    if(robot.flick.getCurrentPosition() > 49){
                        robot.flick.setTargetPosition(0);
                    }
                    setPathState(10);
                }
                break;
//            case 2:
//                if (!follower.isBusy()){
//                    follower.followPath(score1);
//                    setPathState(10);
//                }
//                break;
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

        robot.shooterMotor.setVelocity(1580);




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
}
