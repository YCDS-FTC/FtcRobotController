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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.finalizedSubsystems.intakeSubSystem;

import org.firstinspires.ftc.teamcode.Hardware.HackinHoundsHardware;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;

@Autonomous(name = "DING DONG blue", group = "Examples")
public class DINGDONG_BLUE extends CommandOpMode {

    private HackinHoundsHardware robot = new HackinHoundsHardware();

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;


    private intakeSubSystem intakeSubsystem;


    private final Pose startPose = new Pose(19.794642857142858, 122.14285714285714, Math.toRadians(145));
    private final Pose scorePose = new Pose(59.651785714285715, 84.21428571428572, Math.toRadians(135));
    private final Pose pickupOne = new Pose(125,83, Math.toRadians(0));
    private final Pose pickupTwo = new Pose (126,60, Math.toRadians(0));
    private final Pose curve1 = new Pose(95, 55);
    private final Pose pickupThree = new Pose(125, 35, Math.toRadians(0));
    private final Pose curve2 = new Pose(79, 36);
    private final Pose move = new Pose (60, 60, Math.toRadians(180));



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

    private InstantCommand intakeOn(){
        return new InstantCommand(() ->{
            intakeSubsystem.intakeOn();
        });
    }






    public void autonomousPathUpdate(){
        switch (pathState) {



        }

    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


    @Override
    public void initialize(){



        robot.limelight.pipelineSwitch(0);



        intakeSubsystem  = new intakeSubSystem(hardwareMap, "intake");


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);


    }

    @Override
    public void run(){

    }

}
