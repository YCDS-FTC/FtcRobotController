package org.firstinspires.ftc.teamcode.NewAutosFixedPinpoint;


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

@Autonomous(name="15 close blue", group = "examples")
public class ball15_blue extends OpMode {
    
    private HackinHoundsHardware robot = new HackinHoundsHardware();

    private Follower follower;
    private int pathState;
    private Timer pathTimer, opmodeTimer;

    private final Pose startPose = new Pose(15.73952124789363, 113.74984855724483, Math.toRadians(0));

    private final Pose scoreposecurve = new Pose(40.39822236015184, 101.58067250989289);

    private final Pose scorePose1 = new Pose(49.682310469314075, 84.51985559566788, Math.toRadians(180));

    private final Pose spike1 = new Pose(17.39350180505415, 84.51985559566788, Math.toRadians(180));

    private final Pose opengate = new Pose(11.420743034055726, 60.15479876160991, Math.toRadians(145));

    private final Pose gateControl = new Pose(39.54703758759822, 65.87912284427355);

    private final Pose score3control = new Pose(39.26375585385208, 64.54630550681225);

    private final Pose gatepick = new Pose(12.56656346749225, 54.36842105263156, Math.toRadians(130));

    private final Pose spike2 = new Pose(9.801444043321297, 58.472924187725624, Math.toRadians(215));

    private final Pose spike2control = new Pose(63.36539772663769, 51.74026779626918);

    private final Pose score2control = new Pose(40.54992679192139, 61.23013602172772);

    private final Pose spike3 = new Pose(9.866425992779789, 35.71480144404332, Math.toRadians(180));

    private final Pose spike3control = new Pose(72.15523465703971, 25.14620938628158);

    private final Pose scorePose2 = new Pose(58.62538699690402, 108.55727554179568, Math.toRadians(235));

    private final Pose park = new Pose(27.956678700361007, 79.1696750902527, Math.toRadians(180));

    boolean goodTrack;
    public static Pose endAutoPose;

    private  double turret_tPERd = 4.233;
    private  double angleWant = 125;

    public static double p = 0.03;
    public static double i = 0;
    public static double d = 0.0004;
    public static double f = 0.0001;
    public static double ks = 50;

    PIDFController turretController = new PIDFController(p,i,d,f);


    public static double kp = 14;
    public static double ki = 0;
    public static double kd = 3;
    public static double kf = 1;


    PIDFController shooterController = new PIDFController(kp, ki, kd, kf);

    public static double hoodAngle = 0;

    double Turrettarget = 134;

    public static double shootertarget = 0;


    public double goalX = -0.5;
    public double goalY = 144;

    private Path score1, score2, spikemark2, score3, gate, pickupfromgate, score4, score5, spikemark1, spikemark3, move;

    public void buildPaths(){


        score1 = new Path(new BezierCurve(startPose, scoreposecurve, scorePose1));
        score1.setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading(), 0.5);
        score1.setBrakingStart(6);
        score1.setBrakingStrength(1.5);

        spikemark1 = new Path(new BezierLine(scorePose1, spike1));
        spikemark1.setConstantHeadingInterpolation(spike1.getHeading());


        gate = new Path(new BezierCurve(scorePose1, gateControl, opengate));
        gate.setLinearHeadingInterpolation(scorePose1.getHeading(), opengate.getHeading());
        gate.setBrakingStart(6);
        gate.setBrakingStrength(1.5);

        pickupfromgate = new Path(new BezierLine(opengate, gatepick));
        pickupfromgate.setLinearHeadingInterpolation(opengate.getHeading(), gatepick.getHeading());


        score2 = new Path(new BezierCurve(spike2, score2control, scorePose1));
        score2.setLinearHeadingInterpolation(spike2.getHeading(), scorePose1.getHeading(), 0.8);
        score2.setBrakingStart(6);
        score2.setBrakingStrength(1.5);


        score3 = new Path(new BezierCurve(gatepick, score3control, scorePose1));
        score3.setLinearHeadingInterpolation(gatepick.getHeading(), scorePose1.getHeading(), 0.8);

        spikemark2 = new Path(new BezierCurve(scorePose1, spike2control, spike2));
        spikemark2.setConstantHeadingInterpolation(scorePose1.getHeading());

        score4 = new Path(new BezierLine(spike1, scorePose1));
        score4.setLinearHeadingInterpolation(spike1.getHeading(), scorePose1.getHeading(), 0.8);
        score4.setBrakingStart(6);
        score4.setBrakingStrength(1.5);

        spikemark3 = new Path(new BezierCurve(scorePose1, spike3control, spike3));
        spikemark3.setConstantHeadingInterpolation(spike3.getHeading());

        score5 = new Path(new BezierLine(spike3, scorePose2));
        score5.setLinearHeadingInterpolation(scorePose2.getHeading(), scorePose2.getHeading(), 0.8);
        score5.setBrakingStart(6);
        score5.setBrakingStrength(1.5);

        move = new Path(new BezierLine(scorePose1, park));
        move.setLinearHeadingInterpolation(scorePose1.getHeading() ,park.getHeading());

    }


        public void autonomousPathUpdate(){
            switch (pathState){
                case 0:
                    if (!follower.isBusy()){
                        follower.followPath(score1, true);
                        robot.stopper.setPosition(0.7);
                        robot.intake.setPower(0.3);
                        robot.intake2.setPower(-0.3);
                        shootertarget = 1100;
                        hoodAngle = 0.11;
                        goodTrack = true;
                        setPathState(100);
                    }
                    break;

                case 100:
                    if (!follower.isBusy()){
                        goodTrack = false;
                        if( pathTimer.getElapsedTimeSeconds() > 3){
                            robot.stopper.setPosition(0.47);
                            robot.intake.setPower(1);
                            robot.intake2.setPower(-1);
                            setPathState(1);

                        }
                    }
                    break;

                case 1:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5){
                        goodTrack = true;
                        follower.followPath(spikemark2);
                        robot.stopper.setPosition(0.7);
                        robot.intake.setPower(0.7);
                        robot.intake2.setPower(-0.7);
                        setPathState(24);
                    }
                    break;

                case 24:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5){
                        follower.followPath(score2, true);
                        robot.stopper.setPosition(0.7);
                        robot.intake.setPower(0.7);
                        robot.intake2.setPower(-0.7);
                        setPathState(26);
                    }
                    break;

                case 26:
                    if (!follower.isBusy()){
                        goodTrack = false;
                        if(pathTimer.getElapsedTimeSeconds() > 2.6){
                            robot.stopper.setPosition(0.47);
                            robot.intake.setPower(1);
                            robot.intake2.setPower(-1);
                            setPathState(67);
                        }

                    }
                    break;




                case 67:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.4){
                        goodTrack = true;
                        robot.stopper.setPosition(0.7);
                        follower.followPath(gate);
                        robot.intake.setPower(0.7);
                        robot.intake2.setPower(-0.4);
                        setPathState(89);
                    }
                    break;


                case 89:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.2){
                        follower.followPath(pickupfromgate);
                        robot.intake.setPower(0.7);
                        robot.intake2.setPower(-0.6);
                        setPathState(2);
                    }
                    break;



                case 2:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.5){
                        follower.followPath(score3, true);
                        setPathState(101);
                    }
                    break;

                case 101:
                    if (!follower.isBusy()){
                        goodTrack = false;
                        if(pathTimer.getElapsedTimeSeconds() > 2.6){
                            robot.stopper.setPosition(0.47);
                            robot.intake.setPower(1);
                            robot.intake2.setPower(-1);
                            setPathState(3);

                        }
                    }
                    break;




                case 3:
                    if (!follower.isBusy() & pathTimer.getElapsedTimeSeconds() > 0.5){
                        goodTrack = true;
                        robot.stopper.setPosition(0.7);
                        follower.followPath(spikemark1);
                        setPathState(4);
                    }
                    break;

                case 4:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5){
                        robot.stopper.setPosition(0.7);
                        robot.intake.setPower(0.7);
                        robot.intake2.setPower(-0.7);
                        follower.followPath(score4, true);
                        setPathState(103);
                    }
                    break;

                case 103:
                    if (!follower.isBusy()){
                        goodTrack = false;
                        if(pathTimer.getElapsedTimeSeconds() > 3){
                            robot.intake.setPower(1);
                            robot.intake2.setPower(-1);
                            robot.stopper.setPosition(0.47);
                            setPathState(7);
                        }
                    }
                    break;



                case 7:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5){
                        goodTrack = false;
                        robot.stopper.setPosition(0.7);
                        robot.intake.setPower(0.7);
                        robot.intake2.setPower(-0.7);
                        follower.followPath(spikemark3);
                        setPathState(9);
                    }
                    break;

                case 9:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5){
                        robot.stopper.setPosition(0.7);
                        robot.intake.setPower(0.7);
                        robot.intake2.setPower(-0.7);
                        follower.followPath(score5, true);
                        shootertarget = 1080;
                        Turrettarget = 93;
                        setPathState(10);
                    }
                    break;


                case 10:
                    if (!follower.isBusy()){
                        goodTrack = false;
                        if(pathTimer.getElapsedTimeSeconds() > 3){
                            robot.intake.setPower(1);
                            robot.intake2.setPower(-1);
                            robot.stopper.setPosition(0.47);
                            setPathState(41);
                        }
                    }
                    break;

//                case 41:
//                    if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()> 0.5){
//                        follower.followPath(move);
//                        goodTrack = false;
//                        robot.stopper.setPosition(0.7);
//                        robot.intake2.setPower(0);
//                        robot.intake.setPower(0);
//                    }


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

        double shooterVelocity = robot.shooter.getVelocity();
        double output = shooterController.calculate(shooterVelocity, shootertarget);
        robot.shooter.setVelocity(output);
        robot.angleServo.setPosition(hoodAngle);


        LLResult result = robot.limelight.getLatestResult();
        double tx = 0;


        if(result.isValid() && !goodTrack){
            double target = normA(Turrettarget - result.getTx() -1);
            double turretPosition = robot.turret.getCurrentPosition()/4.233;
            if (target > 150) {target = 150;} else if (target < -150) {target = -150;}

            robot.turret.setVelocity(turretController.calculate(turretPosition, target)* 1450 - robot.imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate * turret_tPERd);


        } else{
            double target = normA(Turrettarget);
            if (target > 150) {target = 150;} else if (target < -150) {target = -150;}
            double turretPosition = robot.turret.getCurrentPosition()/4.233;
            robot.turret.setVelocity(turretController.calculate(turretPosition, target) * 1440);
        }


        telemetry.addData("shooter", robot.shooter.getVelocity());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (Radians)", follower.getPose().getHeading());
        telemetry.addData("Heading (Degrees)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
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
    public void start(){

        robot.limelight.start();
    }

    @Override
    public void stop(){

        endAutoPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());

        RobotPose.endPose = endAutoPose;
        robot.limelight.stop();


    }

    public double normA(double angle) {angle %= 360; if (angle < -180) angle += 360; else if (angle > 180) angle -= 360;return angle;}

}
