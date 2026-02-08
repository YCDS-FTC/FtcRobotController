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

@Autonomous(name="bluefar-w/spikemark", group = "examples")
public class blue_far_wspikemark extends OpMode {

    /**CHECK START POSE **/
    private HackinHoundsHardware robot = new HackinHoundsHardware();

    private Follower follower;
    private int pathState;
    private Timer pathTimer, opmodeTimer;

    private final Pose startPose = new Pose(86.87364620938628, 7.949458483754507, Math.toRadians(0));

    private final Pose scorePose1 = new Pose(51.56317689530685, 17.17689530685918, Math.toRadians(0));

    private final Pose loadingzone = new Pose(12.259927797833933, 10.830324909747317, Math.toRadians(0));

    private final Pose back = new Pose(20.877256317689532, 11.776173285198558, Math.toRadians(0));

    private final Pose loadingzone2 = new Pose(12.02527075812274, 8.17328519855597, Math.toRadians(0));

    private final Pose spike3 = new Pose(15.646209386281608, 36.870036101083045, Math.toRadians(0));

    private final Pose spike3control = new Pose(63.24909747292419, 38.81588447653429);


    private final Pose cycle = new Pose(12.259927797833933, 12.830324909747317);

    private final Pose park = new Pose(101.52707581227436, 16.989169675090256, Math.toRadians(0));

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

    public static double shootertarget = 0;


    public double goalX = 144;
    public double goalY = 144;

    private Path score1, loading, goback, actualpick, score2, spikemark3, score3, cycle1, cycle2, score4, score5, move;

    public void buildPaths(){


        score1 = new Path(new BezierLine(startPose, scorePose1));
        score1.setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading(), 0.5);
        score1.setBrakingStart(6);
        score1.setBrakingStrength(1.5);


        loading = new Path(new BezierLine(scorePose1, loadingzone));
        loading.setLinearHeadingInterpolation(scorePose1.getHeading(), loadingzone.getHeading(), 0.5);
        loading.setBrakingStart(6);
        loading.setBrakingStrength(1.5);

        goback  = new Path(new BezierLine(loadingzone, back));
        goback.setLinearHeadingInterpolation(loadingzone.getHeading(), back.getHeading());
        goback.setBrakingStart(6);
        goback.setBrakingStrength(1.5);


        actualpick = new Path(new BezierLine(back, loadingzone2));
        actualpick.setLinearHeadingInterpolation(back.getHeading(), loadingzone2.getHeading());
        actualpick.setBrakingStart(6);
        actualpick.setBrakingStrength(1.5);


        score2 = new Path(new BezierLine(loadingzone2, scorePose1));
        score2.setLinearHeadingInterpolation(loadingzone2.getHeading(), scorePose1.getHeading());
        score2.setBrakingStart(6);
        score2.setBrakingStrength(1.5);

        spikemark3 = new Path(new BezierCurve(scorePose1, spike3control, spike3));
        spikemark3.setConstantHeadingInterpolation(spike3.getHeading());


        score3 = new Path(new BezierLine(spike3, scorePose1));
        score3.setLinearHeadingInterpolation(spike3.getHeading(), scorePose1.getHeading());
        score3.setBrakingStart(6);
        score3.setBrakingStrength(1.5);

        cycle1 = new Path(new BezierLine(scorePose1, loadingzone2));
        cycle1.setLinearHeadingInterpolation(scorePose1.getHeading(), loadingzone2.getHeading());

        score4 = new Path(new BezierCurve(spike3, spike3control, scorePose1));
        score4.setConstantHeadingInterpolation(scorePose1.getHeading());
        score4.setBrakingStart(6);
        score4.setBrakingStrength(1.5);

        cycle2 = new Path(new BezierLine(scorePose1, cycle));
        cycle2.setLinearHeadingInterpolation(scorePose1.getHeading(), cycle.getHeading());

        score5 = new Path(new BezierLine(cycle, scorePose1));
        score5.setLinearHeadingInterpolation(cycle.getHeading(), scorePose1.getHeading());
        score5.setBrakingStart(6);
        score5.setBrakingStrength(1.5);


        move = new Path(new BezierLine(scorePose1, park));
        move.setConstantHeadingInterpolation(scorePose1.getHeading());


    }


        public void autonomousPathUpdate(){
            switch (pathState){

                case 0:
                    if (!follower.isBusy()) {
                        robot.stopper.setPosition(0.7);
                        follower.setMaxPower(0.7);
                        robot.intake.setPower(0.4);
                        follower.followPath(score1);
                        shootertarget = 1500;
                        hoodAngle = .12;

                        setPathState(1);

                    }
                    break;

                case 1:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 4) {
                        robot.intake.setPower(1);
                        robot.intake2.setPower(-1);
                        robot.stopper.setPosition(0.47);

                        setPathState(7);
                    }
                    break;


                case 7:
                    if(pathTimer.getElapsedTimeSeconds() > 0.5){
                        follower.setMaxPower(1);
                        follower.followPath(loading, true);
                        robot.intake.setPower(0.7);
                        robot.intake2.setPower(-0.7);
                        robot.stopper.setPosition(0.7);
                        setPathState(9);



                    }
                    break;

                case 8:
                    if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5){

                        robot.intake.setPower(0.7);
                        robot.intake2.setPower(-0.7);
                        follower.setMaxPower(1);
                        follower.followPath(goback, true);
                        setPathState(9);

                    }
                    break;

                case 9:
                    if(!follower.isBusy()){

                        robot.intake.setPower(0.7);
                        robot.intake2.setPower(-0.7);
                        follower.setMaxPower(1);
                        follower.followPath(actualpick, true);

                            setPathState(11);


                    }
                    break;





                case 11:
                    if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1){
                        robot.intake2.setPower(-1);
                        robot.intake.setPower(1);
                        follower.setMaxPower(1);
                        follower.followPath(score2, true);
                        setPathState(12);

                    }
                    break;


                case 12:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                        robot.intake.setPower(0.7);
                        robot.intake2.setPower(-0.7);
                        robot.stopper.setPosition(0.47);

                        setPathState(13);
                    }
                    break;


                case 13:
                    if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5){
                        follower.setMaxPower(1);
                        follower.followPath(spikemark3, true);
                        robot.intake.setPower(0.7);
                        robot.intake2.setPower(-0.7);
                        robot.stopper.setPosition(0.7);
                        setPathState(14);

                    }
                    break;


                case 14:
                    if(!follower.isBusy()){
                        robot.intake.setPower(0.7);
                        robot.intake2.setPower(-0.7);
                        robot.stopper.setPosition(0.7);
                        follower.followPath(score3);
                        setPathState(15);

                    }
                    break;

                case 15:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                        robot.intake.setPower(1);
                        robot.intake2.setPower(-1);
                        robot.stopper.setPosition(0.47);

                        setPathState(16);
                    }
                    break;

                case 16:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                        robot.intake.setPower(0.7);
                        robot.intake2.setPower(-0.7);
                        robot.stopper.setPosition(0.7);
                        follower.setMaxPower(0.8);
                        follower.followPath(cycle1);


                        pathTimer.resetTimer();
                        if (pathTimer.getElapsedTimeSeconds() > 2){
                            setPathState(17);
                        }

                        setPathState(17);
                    }
                    break;

                case 17:
                    if (!follower.isBusy()) {
                        robot.intake.setPower(0.4);
                        robot.intake2.setPower(-0.4);
                        robot.stopper.setPosition(0.7);
                        follower.followPath(score4);
                        setPathState(18);
                    }
                    break;


                case 18:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                        robot.intake.setPower(1);
                        robot.intake2.setPower(-1);
                        robot.stopper.setPosition(0.47);

                        setPathState(19);
                    }
                    break;

                case 19:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                        robot.intake.setPower(0.7);
                        robot.intake2.setPower(-0.7);
                        robot.stopper.setPosition(0.7);
                        follower.followPath(cycle2);

                        setPathState(20);
                    }
                    break;

                case 20:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.9) {
                        robot.intake.setPower(0.4);
                        robot.intake2.setPower(-0.4);
                        robot.stopper.setPosition(0.7);
                        follower.followPath(score5);




                        setPathState(21);
                    }
                    break;


                case 21:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                        robot.intake.setPower(1);
                        robot.intake2.setPower(-1);
                        robot.stopper.setPosition(0.47);
                        setPathState(22);
                    }
                    break;


                case 22:
                    if(pathTimer.getElapsedTimeSeconds() > 0.5){
                        robot.intake.setPower(0);
                        robot.intake2.setPower(0);
                        robot.stopper.setPosition(0.7);
                        follower.followPath(move);
                        goodTrack = false;
                        hoodAngle = 0;
                        setPathState(90);
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
