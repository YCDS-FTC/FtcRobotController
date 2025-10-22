package org.firstinspires.ftc.teamcode.PedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Generated Path Auto", group = "Examples")
public class Testing extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private PathChain generatedPath;

    private final Pose startPose = new Pose(26.100, 130.400, Math.toRadians(-36));

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // ---- GENERATED PATH BUILT DIRECTLY HERE ----
        generatedPath = follower.pathBuilder()
                .addPath(
                        // Path 1
                        new BezierLine(
                                new Pose(26.100, 130.400),
                                new Pose(44.602, 84.584)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-36), Math.toRadians(180))
                .addPath(
                        // Path 2
                        new BezierCurve(
                                new Pose(44.602, 84.584),
                                new Pose(-16.575, 85.000),
                                new Pose(72.637, 74.708)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Path 3
                        new BezierCurve(
                                new Pose(72.637, 74.708),
                                new Pose(65.947, 59.575),
                                new Pose(34.885, 60.053)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Path 4
                        new BezierCurve(
                                new Pose(34.885, 60.053),
                                new Pose(-16.071, 62.442),
                                new Pose(76.938, 46.673),
                                new Pose(72.000, 21.823)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Path 5
                        new BezierCurve(
                                new Pose(72.000, 21.823),
                                new Pose(-30.894, 53.363),
                                new Pose(72.796, 20.071)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        // ---- END GENERATED PATH ----
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Ready to run Generated Path Auto");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        pathState = 0;

        follower.followPath(generatedPath);
    }

    @Override
    public void loop() {
        follower.update();

        if (!follower.isBusy() && pathState == 0) {
            pathState = 1;
            //follower.holdPoint(); // optional: lock final pose
        }

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void stop() {
        // optional cleanup
    }
}
