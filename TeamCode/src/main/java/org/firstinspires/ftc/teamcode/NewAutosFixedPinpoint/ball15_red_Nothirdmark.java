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
import org.firstinspires.ftc.teamcode.Hardware.HackinHoundsHardware;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.RobotPose;

@Autonomous(name="15 close red-without third spike mark", group = "examples")
public class ball15_red_Nothirdmark extends OpMode {

    /**CHECK START POSE **/
    private HackinHoundsHardware robot = new HackinHoundsHardware();

    private Follower follower;
    private int pathState;
    private Timer pathTimer, opmodeTimer;

    private final Pose startPose = new Pose(128.6226144817707, 114.00848917728916, Math.toRadians(180));

    private final Pose scorePose1 = new Pose(89.44404332129963, 83.48014440433212, Math.toRadians(0));

    private final Pose scoreCurve = new Pose(101.78255490772712, 105.3263601344639);

    private final Pose spike1 = new Pose(126.77325613886062, 83.48014440433212, Math.toRadians(0));

    private final Pose opengate = new Pose(131.69586458182005, 60.88747527131697, Math.toRadians(30));

    private final Pose gateControl = new Pose(103.26632651920735, 46.38423064456641);

    private final Pose pickgate = new Pose(129.10216718266255, 55.24767801857584, Math.toRadians(48));

    private final Pose score3control = new Pose(103.98363156777057, 66.12242514334254);

    private final Pose spike2 = new Pose(133.35184513883854, 59.58152686145146, Math.toRadians(330));

    private final Pose spike2control = new Pose(81.89167293303436, 54.50161791375891);


    private final Pose spike3 = new Pose(130.07462686567163, 34.652452025586356, Math.toRadians(0));

    private final Pose spike3control = new Pose(77.64313040265408, 24.97248158382918);

    private final Pose scorePose2 = new Pose(85.52330028724391, 107.84825585944049, Math.toRadians(330));

    private final Pose park = new Pose(114.56289978678038, 83.33901918976545, Math.toRadians(0));

    boolean goodTrack;
    public static Pose endAutoPose;

    private  double turret_tPERd = 4.233;
    private  double angleWant = 125;

    public static double p = 0.03;
    public static double i = 0;
    public static double d = 0.0003;
    public static double f = 0;
    public static double ks = 50;

    PIDFController turretController = new PIDFController(p,i,d,f);

    double Turrettarget = -134;


    private double target;
    public static double kp = 14;
    public static double ki = 0;
    public static double kd = 3;
    public static double kf = 1;


    PIDFController shooterController = new PIDFController(kp, ki, kd, kf);

    public static double hoodAngle = 0;

    public static double shootertarget = 0;


    public double goalX = 144;
    public double goalY = 144;

    private Path score1, score2, spikemark2, score3, score4, gate, pickfromgate, spikemark1, spikemark3, move, score5;

    public void buildPaths(){


        score1 = new Path(new BezierCurve(startPose, scoreCurve, scorePose1));
        score1.setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading(), 0.5);
        score1.setBrakingStart(6);
        score1.setBrakingStrength(1.5);

        spikemark1 = new Path(new BezierLine(scorePose1, spike1));
        spikemark1.setConstantHeadingInterpolation(spike1.getHeading());


        score2 = new Path(new BezierLine(spike2, scorePose1));
        score2.setBrakingStart(6);
        score2.setBrakingStrength(1.5);
        score2.setLinearHeadingInterpolation(spike2.getHeading(), scorePose1.getHeading());

        gate = new Path(new BezierCurve(scorePose1, gateControl, opengate));
        gate.setLinearHeadingInterpolation(spike1.getHeading(), opengate.getHeading());
        gate.setBrakingStart(6);
        gate.setBrakingStart(1.5);

        pickfromgate = new Path(new BezierLine(opengate, pickgate));
        pickfromgate.setLinearHeadingInterpolation(opengate.getHeading() , pickgate.getHeading());


        score3 = new Path(new BezierCurve(pickgate, score3control, scorePose1));
        score3.setLinearHeadingInterpolation(pickgate.getHeading(), scorePose1.getHeading());
        score3.setBrakingStart(6);
        score3.setBrakingStrength(1.5);


        spikemark2 = new Path(new BezierCurve(scorePose1, spike2control, spike2));
        spikemark2.setConstantHeadingInterpolation(scorePose1.getHeading());

        score4 = new Path(new BezierCurve(pickgate, score3control, scorePose1));
        score4.setLinearHeadingInterpolation(pickgate.getHeading(), scorePose1.getHeading());
        score4.setBrakingStart(6);
        score4.setBrakingStrength(1.5);


        spikemark3 = new Path(new BezierCurve(scorePose1, spike3control, spike3));
        spikemark3.setConstantHeadingInterpolation(spike3.getHeading());

        score5 = new Path(new BezierLine(spike1, scorePose2));
        score5.setLinearHeadingInterpolation(spike1.getHeading(), scorePose2.getHeading(), 0.8);
        score5.setBrakingStart(6);
        score5.setBrakingStrength(1.5);

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
                        shootertarget = 1110;
                        hoodAngle = 0.11;
                        goodTrack = true;
                        setPathState(100);
                    }
                    break;

                case 100:
                    if (!follower.isBusy()) {
                        goodTrack = false;
                        if(pathTimer.getElapsedTimeSeconds() > 3){
                            robot.stopper.setPosition(0.47);
                            robot.intake.setPower(0.7);
                            robot.intake2.setPower(-0.7);
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
                        follower.followPath(score2);
                        robot.stopper.setPosition(0.7);
                        robot.intake.setPower(0.7);
                        robot.intake2.setPower(-0.7);
                        setPathState(26);
                    }
                    break;

                case 26:
                    if (!follower.isBusy()) {
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
                        pathTimer.resetTimer();
                        robot.stopper.setPosition(0.7);
                        follower.followPath(gate);
                        robot.intake.setPower(0.7);
                        robot.intake2.setPower(-0.4);
                        setPathState(89);

                    }
                    break;


                case 89:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.2){
                        follower.followPath(pickfromgate);
                        robot.intake.setPower(0.7);
                        robot.intake2.setPower(-0.6);
                        setPathState(2);
                    }
                    break;



                case 2:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.5){
                        follower.followPath(score3);
                        setPathState(101);
                    }
                    break;

                case 101:
                    if (!follower.isBusy()){
                        goodTrack = false;
                        if(pathTimer.getElapsedTimeSeconds() > 3){
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
                        follower.followPath(gate);
                        setPathState(80);
                    }
                    break;

                case 80:
                    if (!follower.isBusy() & pathTimer.getElapsedTimeSeconds() > 1){
                        goodTrack = true;
                        robot.stopper.setPosition(0.7);
                        follower.followPath(pickfromgate);
                        setPathState(4);
                    }
                    break;

                case 4:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1){
                        robot.stopper.setPosition(0.7);
                        robot.intake.setPower(0.7);
                        robot.intake2.setPower(-0.7);
                        follower.followPath(score4);
                        setPathState(103);
                    }
                    break;

                case 103:
                    if (!follower.isBusy()){
                        goodTrack = false;
                        if( pathTimer.getElapsedTimeSeconds() > 2.6){
                            robot.intake.setPower(1);
                            robot.intake2.setPower(-1);
                            robot.stopper.setPosition(0.47);
                            setPathState(7);
                        }
                    }
                    break;



                case 7:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5){
                        goodTrack = true;
                        robot.stopper.setPosition(0.7);
                        robot.intake.setPower(0.7);
                        robot.intake2.setPower(-0.7);
                        follower.followPath(spikemark1);
                        setPathState(9);
                    }
                    break;

                case 9:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5){
                        Turrettarget = -90;
                        robot.stopper.setPosition(0.7);
                        robot.intake.setPower(0.7);
                        robot.intake2.setPower(-0.7);
                        follower.followPath(score5);
                        shootertarget = 1080;
                        setPathState(10);
                    }
                    break;


                case 10:
                    if (!follower.isBusy()){
                        goodTrack = false;
                        goalX = 145;
                        if (pathTimer.getElapsedTimeSeconds() > 3) {
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

        LLResult result = robot.limelight.getLatestResult();
        double tx = 0;

        double shooterVelocity = robot.shooter.getVelocity();
        double output = shooterController.calculate(shooterVelocity, shootertarget);
        robot.shooter.setVelocity(output);
        robot.angleServo.setPosition(hoodAngle);
//



        if(result.isValid() && !goodTrack){
            double target = normA(Turrettarget - result.getTx() - 0.5);
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
        robot.limelight.pipelineSwitch(1);





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
