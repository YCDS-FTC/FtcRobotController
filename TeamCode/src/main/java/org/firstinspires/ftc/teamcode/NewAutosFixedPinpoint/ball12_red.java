package org.firstinspires.ftc.teamcode.NewAutosFixedPinpoint;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.HackinHoundsHardware;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.RobotPose;

@Autonomous(name="12 close red", group = "examples")
public class ball12_red extends OpMode {
    
    private HackinHoundsHardware robot = new HackinHoundsHardware();

    private Follower follower;
    private int pathState;
    private Timer pathTimer, opmodeTimer;

    private final Pose startPose = new Pose(112.57562554462842, 134.62490304417355, Math.toRadians(270));

    private final Pose scorePose1 = new Pose(89.44404332129963, 83.48014440433212, Math.toRadians(0));

    private final Pose spike1 = new Pose(126.42960288808663, 83.48014440433212, Math.toRadians(0));

    private final Pose emptyGate = new Pose(128.75090252707582, 72.43321299638991, Math.toRadians(90));

    private final Pose gateControl = new Pose(123.03068592057762, 77.78339350180505);
    private final Pose spike2 = new Pose(125.81338360037701, 59.58152686145146, Math.toRadians(0));

    private final Pose spike2control = new Pose(82.89167293303436, 56.50161791375891);


    private final Pose spike3 = new Pose(131.07462686567163, 34.652452025586356, Math.toRadians(0));

    private final Pose spike3control = new Pose(77.64313040265408, 28.97248158382918);

    private final Pose park = new Pose(114.56289978678038, 83.33901918976545, Math.toRadians(0));

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


    public static double kp = 11;
    public static double ki = 0;
    public static double kd = 2;
    public static double kf = 1;


    PIDFController shooterController = new PIDFController(kp, ki, kd, kf);

    public static double hoodAngle = 0;

    public static double shootertarget = 0;


    public double goalX = 143;
    public double goalY = 143;

    private Path score1, score2, spikemark2, score3, gate, score4,spikemark1, spikemark3, move;

    public void buildPaths(){


        score1 = new Path(new BezierLine(startPose, scorePose1));
        score1.setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading(), 0.5);
        score1.setBrakingStart(6);
        score1.setBrakingStrength(1.5);

        spikemark1 = new Path(new BezierLine(scorePose1, spike1));
        spikemark1.setConstantHeadingInterpolation(spike1.getHeading());


        gate = new Path(new BezierCurve(spike1, gateControl, emptyGate));
        gate.setLinearHeadingInterpolation(spike1.getHeading(), emptyGate.getHeading());

        score2 = new Path(new BezierLine(emptyGate, scorePose1));
        score2.setLinearHeadingInterpolation(emptyGate.getHeading(), scorePose1.getHeading());
        score2.setBrakingStart(6);
        score2.setBrakingStrength(1.5);


        spikemark2 = new Path(new BezierCurve(scorePose1, spike2control, spike2));
        spikemark2.setConstantHeadingInterpolation(spike2.getHeading());

        score3 = new Path(new BezierLine(spike2, scorePose1));
        score3.setConstantHeadingInterpolation(spike2.getHeading());
        score3.setBrakingStart(6);
        score3.setBrakingStrength(1.5);

        spikemark3 = new Path(new BezierCurve(scorePose1, spike3control, spike3));
        spikemark3.setConstantHeadingInterpolation(spike3.getHeading());

        score4 = new Path(new BezierLine(spike3, scorePose1));
        score4.setConstantHeadingInterpolation(scorePose1.getHeading());
        score4.setBrakingStart(6);
        score4.setBrakingStrength(1.5);

        move = new Path(new BezierLine(scorePose1, park));
        move.setConstantHeadingInterpolation(scorePose1.getHeading());


    }


        public void autonomousPathUpdate(){
            switch (pathState){

                case 0:
                    if (!follower.isBusy()){
                        follower.followPath(score1);
                        robot.stopper.setPosition(0.7);
                        robot.intake.setPower(0.3);
                        robot.intake2.setPower(-0.3);
                        shootertarget = 1100;
                        hoodAngle = 0;
                        goodTrack = true;
                        setPathState(100);
                    }
                    break;

                case 100:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.5){
                        robot.stopper.setPosition(0.47);
                        robot.intake.setPower(1);
                        robot.intake2.setPower(-1);
                        setPathState(1);
                    }
                    break;

                case 1:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5){
                        follower.followPath(spikemark1);
                        robot.stopper.setPosition(0.7);
                        robot.intake.setPower(0.7);
                        robot.intake2.setPower(-0.7);
                        setPathState(67);
                    }
                    break;



                case 67:
                    if (!follower.isBusy()){
                       follower.followPath(gate);
                        setPathState(2);
                    }
                    break;

                case 2:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.5){
                        follower.followPath(score2);
                        setPathState(101);
                    }
                    break;

                case 101:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3){
                        robot.stopper.setPosition(0.47);
                        robot.intake.setPower(1);
                        robot.intake2.setPower(-1);
                        setPathState(3);
                    }
                    break;




                case 3:
                    if (!follower.isBusy() & pathTimer.getElapsedTimeSeconds() > 0.5){
                        follower.followPath(spikemark2);
                        setPathState(4);
                    }
                    break;

                case 4:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5){
                        robot.stopper.setPosition(0.7);
                        robot.intake.setPower(0.7);
                        robot.intake2.setPower(-0.7);
                        follower.followPath(score3);
                        setPathState(103);
                    }
                    break;

                case 103:
                    if (!follower.isBusy()  && pathTimer.getElapsedTimeSeconds() > 3){
                        robot.intake.setPower(1);
                        robot.intake2.setPower(-1);
                        robot.stopper.setPosition(0.47);
                        setPathState(7);
                    }
                    break;



                case 7:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5){
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
                        follower.followPath(score3);
                        setPathState(10);
                    }
                    break;


                case 10:
                    if (!follower.isBusy()  && pathTimer.getElapsedTimeSeconds() > 3.5){
                        robot.intake.setPower(1);
                        robot.intake2.setPower(-1);
                        robot.stopper.setPosition(0.47);
                        setPathState(41);
                    }
                    break;

                case 41:
                    if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()> 0.5){
                        follower.followPath(move);
                        goodTrack = false;
                        robot.stopper.setPosition(0.7);
                        robot.intake2.setPower(0);
                        robot.intake.setPower(0);
                    }

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

        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();

        double robXV = robot.pinpoint.getVelX(DistanceUnit.INCH) * 0.3;
        double robYV = robot.pinpoint.getVelY(DistanceUnit.INCH) * 0.3;

        double dx = goalX - (robotX + robXV);
        double dy = goalY - (robotY + robYV);

        double goalHeadingField = Math.atan2(-dy, -dx);
        double goalHeadingFieldDegrees = Math.toDegrees(goalHeadingField);

        double robotHeading = follower.getPose().getHeading();
        double robotHeadingDegrees = Math.toDegrees(robotHeading);

        double turretTargetAngle = goalHeadingFieldDegrees - robotHeadingDegrees;
        double turretAngle = robot.turret.getCurrentPosition()/turret_tPERd;

        double target = normA(turretTargetAngle);
        if (target > 150) {target = 150;} else if (target < -150) {target = -150;}

        if(goodTrack){
            double turretPower = (turretController.calculate(turretAngle, target));
            robot.turret.setPower(turretPower);
        } else{
            target = normA(0);
            double turretPower = (turretController.calculate(turretAngle, target));
            robot.turret.setPower(turretPower);
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






        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }



    @Override
    public void start(){

    }

    @Override
    public void stop(){

        endAutoPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());

        RobotPose.endPose = endAutoPose;


    }

    public double normA(double angle) {angle %= 360; if (angle < -180) angle += 360; else if (angle > 180) angle -= 360;return angle;}

}
