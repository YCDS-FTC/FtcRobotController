package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "DING DONG auto", group = "Examples")
public class DINGDONG_autonomous extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

//    private final Pose startPose = new Pose(122, 123,)
    private final Pose scorePose = new Pose(84, 83, Math.toRadians(45));


    @Override
    public void loop(){

    }

    @Override
    public void init(){

    }

    @Override
    public void init_loop(){

    }

    @Override
    public void start(){

    }

    @Override
    public void stop(){

    }
}
