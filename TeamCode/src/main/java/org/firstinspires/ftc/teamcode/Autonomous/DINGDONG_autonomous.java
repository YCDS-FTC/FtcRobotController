package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
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
    private final Pose pickupOne = new Pose(125,83, Math.toRadians(0));
    private final Pose pickupTwo = new Pose (126,60, Math.toRadians(0));
    private final Pose curve1 = new Pose(95, 55);
    private final Pose pickupThree = new Pose(125, 35, Math.toRadians(0));
    private final Pose curve2 = new Pose(79, 36);
    private final Pose move = new Pose (84, 60, Math.toRadians(0));



    private Path scorePreload;

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
