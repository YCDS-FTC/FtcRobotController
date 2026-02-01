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

@Autonomous(name="18ball? LEBRONNNN-test", group = "examples")
public class Ball_LEBRON18bluetesting extends OpMode {
    
    private HackinHoundsHardware robot = new HackinHoundsHardware();

    private Follower follower;
    private int pathState;
    private Timer pathTimer, opmodeTimer;

    private final Pose startPose = new Pose(31.98166982239339, 134.40549905328197, Math.toRadians(270));

    private final Pose scorePose1 = new Pose(53.92057761732854, 75.81588447653431, Math.toRadians(180));

    private final Pose spike2 = new Pose(15.76534296028881, 54.50902527075811, Math.toRadians(180));

    private final Pose spike2control = new Pose(59.4053259778058, 56.60469314079424);

    private final Pose gatepickup = new Pose(11.528792569659443, 60.85448916408669, Math.toRadians(150));

    private final Pose gatereal = new Pose(11.528792569659443, 56.85448916408669, Math.toRadians(150));

    private final Pose gatepickupcurve = new Pose(46.973077551640294, 67.64261766293536);

    private final Pose scorePose2 = new Pose(46.98916967509027, 84.51985559566789, Math.toRadians(180));

    private final Pose spike1 = new Pose(22.555956678700376, 84.22382671480143, Math.toRadians(180));

    private final Pose spike3 = new Pose(21.08664259927798, 34.83032490974729, Math.toRadians(180));

    private final Pose spike3control = new Pose(65.98142414860682, 30.761609907120746);

    private final Pose scorePose3 = new Pose(56.5664328622579, 104.22279815839822, Math.toRadians(250));



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


    public double goalX = 1;
    public double goalY = 144;

    private Path score1, spikemark2, score2, gatepickup1, pickingFromGate, score3, gatepickup2, score4, spikemark1, score5, spikemark3, score6;

    public void buildPaths(){


        score1 = new Path(new BezierLine(startPose, scorePose1));
        score1.setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading(), 0.5);
        score1.setBrakingStart(6);
        score1.setBrakingStrength(1.5);

        spikemark2 = new Path(new BezierCurve(scorePose1, spike2control, spike2));
        spikemark2.setConstantHeadingInterpolation(spike2.getHeading());


        score2 = new Path(new BezierLine(spike2, scorePose1));
        score2.setConstantHeadingInterpolation(spike2.getHeading());
        score2.setBrakingStart(10);
        score2.setBrakingStrength(1.5);

        gatepickup1 = new Path(new BezierLine(scorePose1, gatepickup));
        gatepickup1.setLinearHeadingInterpolation(scorePose1.getHeading(), gatepickup.getHeading(), 0.75);

        pickingFromGate = new Path(new BezierLine(gatepickup, gatereal));
        pickingFromGate.setLinearHeadingInterpolation(gatepickup.getHeading(), gatereal.getHeading());

        score3 = new Path(new BezierLine(gatepickup, scorePose1));
        score3.setLinearHeadingInterpolation(gatepickup.getHeading(), scorePose1.getHeading(), 0.7);
        score3.setBrakingStart(10);
        score3.setBrakingStrength(1.5);

        gatepickup2 = new Path(new BezierLine(scorePose1, gatepickup));
        gatepickup2.setLinearHeadingInterpolation(scorePose1.getHeading(), gatepickup.getHeading(), 0.75);

        score4 = new Path(new BezierCurve(gatepickup, gatepickupcurve, scorePose2));
        score4.setLinearHeadingInterpolation(gatepickup.getHeading(), scorePose2.getHeading(), 0.8);
        score4.setBrakingStart(10);
        score4.setBrakingStrength(1.5);

        spikemark1 = new Path(new BezierLine(scorePose2, spike1));
        spikemark1.setConstantHeadingInterpolation(spike1.getHeading());

        score5 = new Path(new BezierLine(spike1, scorePose2));
        score5.setConstantHeadingInterpolation(scorePose2.getHeading());
        score5.setBrakingStart(6);
        score5.setBrakingStrength(1.5);

        spikemark3 = new Path(new BezierCurve(scorePose2, spike3control, spike3));
        spikemark3.setConstantHeadingInterpolation(spike3.getHeading());

        score6 = new Path(new BezierLine(spike3, scorePose3));
        score6.setConstantHeadingInterpolation(scorePose3.getHeading());
        score6.setBrakingStart(6);
        score6.setBrakingStrength(1.5);

    }


        public void autonomousPathUpdate(){
            switch (pathState){

                case 0:
                    if (!follower.isBusy()){
                        follower.followPath(score1);
                        robot.stopper.setPosition(0.7);
                        robot.intake.setPower(0.3);
                        robot.intake2.setPower(-0.3);
                        shootertarget = 1140;
                        hoodAngle = 0;
                        setPathState(100);
                    }
                    break;

                case 100:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1){
                        robot.stopper.setPosition(0.47);
                        robot.intake.setPower(1);
                        robot.intake2.setPower(-1);
                        setPathState(1);
                    }
                    break;

                case 1:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5){
                        follower.followPath(spikemark2);
                        robot.stopper.setPosition(0.7);
                        robot.intake.setPower(0.7);
                        robot.intake2.setPower(-0.7);
                        setPathState(2);
                    }
                    break;

                case 2:
                    if (!follower.isBusy()){
                        follower.followPath(score2);
                        setPathState(101);
                    }
                    break;

                case 101:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.2){
                        robot.stopper.setPosition(0.47);
                        robot.intake.setPower(1);
                        robot.intake2.setPower(-1);
                        setPathState(3);
                    }
                    break;

                case 3:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5){
                        follower.followPath(gatepickup1);
                        robot.stopper.setPosition(0.7);
                        robot.intake.setPower(0.7);
                        robot.intake2.setPower(-0.7);
                        setPathState(120);
                    }
                    break;

                case 120:
                    if (!follower.isBusy()){
                        follower.followPath(pickingFromGate);
                        robot.stopper.setPosition(0.7);
                        robot.intake.setPower(0.7);
                        robot.intake2.setPower(-0.7);
                        setPathState(4);
                    }
                    break;


                case 4:
                    if (!follower.isBusy() & pathTimer.getElapsedTimeSeconds() > 2){
                        follower.followPath(score3);
                        setPathState(102);
                    }
                    break;

                case 102:
                    if (!follower.isBusy() & pathTimer.getElapsedTimeSeconds() > 2.5){
                        robot.intake.setPower(1);
                        robot.intake2.setPower(-1);
                        robot.stopper.setPosition(0.47);
                        setPathState(5);
                    }
                    break;


                case 5:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()> 0.5){
                        follower.followPath(gatepickup2);
                        robot.stopper.setPosition(0.7);
                        robot.intake.setPower(0.7);
                        robot.intake2.setPower(-0.7);
                        shootertarget = 1120;
                        setPathState(6);
                    }
                    break;

                case 6:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3){
                        follower.followPath(score4);
                        setPathState(103);
                    }
                    break;

                case 103:
                    if (!follower.isBusy()  && pathTimer.getElapsedTimeSeconds() > 2.3){
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
                        follower.followPath(spikemark1);
                        setPathState(8);
                    }
                    break;

                case 8:
                    if (!follower.isBusy()){
                        follower.followPath(score5);
                        setPathState(104);
                    }
                    break;

                case 104:
                if (!follower.isBusy()  && pathTimer.getElapsedTimeSeconds() > 1.7){
                    robot.intake.setPower(1);
                    robot.intake2.setPower(-1);
                    robot.stopper.setPosition(0.47);
                    setPathState(9);
                }
                break;

                case 9:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5){
                        follower.followPath(spikemark3);
                        robot.stopper.setPosition(0.7);
                        robot.intake.setPower(0.7);
                        robot.intake2.setPower(-0.7);
                        shootertarget = 1120;
                        setPathState(10);
                    }
                    break;

                case 10:
                    if (!follower.isBusy()){
                        follower.followPath(score6);
                        setPathState(105);
                    }
                    break;

                case 105:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2){
                        robot.intake.setPower(1);
                        robot.intake2.setPower(-1);
                        robot.stopper.setPosition(0.47);
                        setPathState(110);
                    }
                    break;

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

        double turretPower = (turretController.calculate(turretAngle, target));
        robot.turret.setPower(turretPower);

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
