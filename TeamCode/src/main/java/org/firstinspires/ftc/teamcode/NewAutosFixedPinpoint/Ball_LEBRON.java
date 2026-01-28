package org.firstinspires.ftc.teamcode.NewAutosFixedPinpoint;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.HackinHoundsHardware;

@Autonomous(name="18ball? LEBRONNNN", group = "examples")
public class Ball_LEBRON extends OpMode {
    
    private HackinHoundsHardware robot = new HackinHoundsHardware();

    private Follower follower;
    private int pathState;
    private Timer pathTimer, opmodeTimer;

    private final Pose startPose = new Pose(33.819494584837535, 134.6714801444043, Math.toRadians(270));
    private final Pose scorePose1 = new Pose(53.92057761732854, 74.81588447653431, Math.toRadians(180));

    private final Pose spike2 = new Pose(17.76534296028881, 58.50902527075811, Math.toRadians(180));

    private final Pose spike2control = new Pose(59.4053259778058, 58.60469314079424);
    private final Pose gatepickup = new Pose(13.516245487364626, 60.71841155234658, Math.toRadians(145));

    private final Pose scorePose2 = new Pose(46.98916967509027, 84.51985559566789, Math.toRadians(180));

    private final Pose spike1 = new Pose(22.555956678700376, 84.22382671480143, Math.toRadians(180));

    private final Pose spike3 = new Pose(24.08664259927798, 34.83032490974729, Math.toRadians(180));

    private final Pose spike3control = new Pose(62.50722021660651, 33.94404332129967);

    private final Pose scorePose3 = new Pose(61.722021660649816, 103.49458483754508, Math.toRadians(240));



    private Path score1, spikemark2, score2, gatepickup1, score3, gatepickup2, score4, spikemark1, score5, spikemark3, score6;

    public void buildPaths(){


        score1 = new Path(new BezierLine(startPose, scorePose1));
        score1.setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading(), 0.5);


        spikemark2 = new Path(new BezierCurve(scorePose1, spike2control, spike2));
        spikemark2.setConstantHeadingInterpolation(spike2.getHeading());


        score2 = new Path(new BezierLine(spike2, scorePose1));
        score2.setConstantHeadingInterpolation(spike2.getHeading());

        gatepickup1 = new Path(new BezierLine(scorePose1, gatepickup));
        gatepickup1.setLinearHeadingInterpolation(scorePose1.getHeading(), gatepickup.getHeading(), 0.75);


        score3 = new Path(new BezierLine(gatepickup, scorePose1));
        score3.setLinearHeadingInterpolation(gatepickup.getHeading(), scorePose1.getHeading(), 0.7);

        gatepickup2 = new Path(new BezierLine(scorePose1, gatepickup));
        gatepickup2.setLinearHeadingInterpolation(scorePose1.getHeading(), gatepickup.getHeading(), 0.75);

        score4 = new Path(new BezierLine(gatepickup, scorePose2));
        score4.setLinearHeadingInterpolation(gatepickup.getHeading(), scorePose2.getHeading(), 0.8);

        spikemark1 = new Path(new BezierLine(scorePose2, spike1));
        spikemark1.setConstantHeadingInterpolation(spike1.getHeading());

        score5 = new Path(new BezierLine(spike1, scorePose2));
        score5.setConstantHeadingInterpolation(scorePose2.getHeading());

        spikemark3 = new Path(new BezierCurve(scorePose2, spike3control, spike3control));
        spikemark3.setConstantHeadingInterpolation(spike3.getHeading());

        score6 = new Path(new BezierLine(spike3, scorePose3));
        score6.setConstantHeadingInterpolation(scorePose3.getHeading());


    }


        public void autonomousPathUpdate(){
            switch (pathState){}
        }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }




    @Override
    public void loop(){

    }



    @Override
    public void init(){

    }



    @Override
    public void start(){

    }

    @Override
    public void stop(){

    }
}
