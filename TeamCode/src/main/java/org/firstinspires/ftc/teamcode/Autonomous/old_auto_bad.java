package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Commands.intakeLow;
import org.firstinspires.ftc.teamcode.Commands.shooterOn;
import org.firstinspires.ftc.teamcode.Commands.stopperOn;
import org.firstinspires.ftc.teamcode.Commands.transferLow;
import org.firstinspires.ftc.teamcode.Commands.turretScore;
import org.firstinspires.ftc.teamcode.Commands.turretZero;
import org.firstinspires.ftc.teamcode.finalizedSubsystems.intakeSubSystem;

import org.firstinspires.ftc.teamcode.Hardware.HackinHoundsHardware;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.finalizedSubsystems.shooterSubsystem;
import org.firstinspires.ftc.teamcode.finalizedSubsystems.stopperSubsystem;
import org.firstinspires.ftc.teamcode.finalizedSubsystems.transferSubsystem;
import org.firstinspires.ftc.teamcode.finalizedSubsystems.turretSubsystem;

public class old_auto_bad extends CommandOpMode {

    private HackinHoundsHardware robot = new HackinHoundsHardware();

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private Limelight3A limelight;

    private intakeSubSystem intakeSubsystem;
    private turretSubsystem turretSubsystem;
    private transferSubsystem transferSubsystem;
    private stopperSubsystem stopperSubsystem;
    private shooterSubsystem shooterSubsystem;


    private final Pose startPose = new Pose(17.6580310880829, 121.3678756476684, Math.toRadians(53));
    private final Pose scorePose = new Pose(59.651785714285715, 84.21428571428572, Math.toRadians(180));
    private final Pose pickupOne = new Pose(20,84, Math.toRadians(180));
    private final Pose gateEmpty = new Pose(15.46153846153846, 73, Math.toRadians(90));
    private final Pose goBack = new Pose(40.15384615384615, 78.46153846153847);
    private final Pose pickupTwo = new Pose (17,60, Math.toRadians(180));
    private final Pose curve1 = new Pose(80, 55);
    private final Pose pickupThree = new Pose(15.923076923076923, 35.07692307692308, Math.toRadians(180));
    private final Pose curve2 = new Pose(72.46153846153845, 28.61538461538461);
    private final Pose move = new Pose (22.615384615384613, 84.46153846153847, Math.toRadians(180));



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

    private InstantCommand intakeOff(){
        return new InstantCommand(() ->{
            intakeSubsystem.intakeOff();
        });
    }

    private InstantCommand intakeLow(){
        return new InstantCommand(() ->{
            intakeSubsystem.intakeLow();
        });
    }

    private InstantCommand transferOn(){
        return new InstantCommand(() ->{
            transferSubsystem.transferOn();
        });
    }

    private InstantCommand transferOff(){
        return new InstantCommand(() ->{
            transferSubsystem.transferOff();
        });
    }

    private InstantCommand transferLow(){
        return new InstantCommand(() ->{
            transferSubsystem.transferLow();
        });
    }

    private InstantCommand stopperOn(){
        return new InstantCommand(() ->{
            stopperSubsystem.stopperOn();
        });
    }

    private InstantCommand stopperOff(){
        return new InstantCommand(() ->{
            stopperSubsystem.stopperOff();
        });
    }

    private InstantCommand shooterOff(){
        return new InstantCommand(() ->{
            shooterSubsystem.shooterOff();
        });
    }

    private RunCommand shooterOn() {
        return new RunCommand(() -> {
            shooterSubsystem.periodic();
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
        super.reset();

        robot.init(hardwareMap);

        intakeSubsystem  = new intakeSubSystem(hardwareMap, "intake");
        transferSubsystem = new transferSubsystem(hardwareMap, "intake2");
        stopperSubsystem  = new stopperSubsystem(hardwareMap,"stopper");
        turretSubsystem = new turretSubsystem(hardwareMap, "turret");
        shooterSubsystem = new shooterSubsystem(hardwareMap, "shooter");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        turretScore turretScore = new turretScore(turretSubsystem);
        turretZero turretZero = new turretZero(turretSubsystem);
        shooterOn shooterOn = new shooterOn(shooterSubsystem);
        intakeLow intakeLow = new intakeLow(intakeSubsystem);
        transferLow transferLow = new transferLow(transferSubsystem);
        stopperOn stopperOn = new stopperOn(stopperSubsystem);



        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();
        buildPaths();




        SequentialCommandGroup BlueClose = new SequentialCommandGroup(

                // ----------------------------------------------------------------------
                // STEP 1: START (Run continuous commands in parallel with the main sequence)
                // ----------------------------------------------------------------------
                // Runs until interrupted by turretZero

                        // This SequentialCommandGroup contains ALL of your time-limited pathing/intake steps.
                        // Once this inner group finishes, the outer ParallelCommandGroup is done.

                                // 1. Score Preload Movement/Setup
                                new ParallelCommandGroup(
                                        new FollowPathCommand(follower, scorePreload)


                                ),

                                // 2. Score Preload Sequence
                                new ParallelCommandGroup(
                                        intakeLow(),
                                        transferLow(),
                                        stopperOn()
                                ),
                                intakeOn(),
                                transferOn(),
                                stopperOff(),
                                new WaitCommand(2000), // Time to fire

                                // 3. Pickup 1 Movement
                                new ParallelCommandGroup(
                                        stopperOn(),
                                        new FollowPathCommand(follower, pickup1).setGlobalMaxPower(0.3)
                                ),

                                // 4. Score 1 Movement/Setup
                                new ParallelCommandGroup(
                                        intakeLow(),
                                        transferLow(),
                                        new FollowPathCommand(follower, score1).setGlobalMaxPower(1)
                                ),

                                // 5. Score 1 Sequence
                                new ParallelCommandGroup(
                                        intakeOn(),
                                        transferOn()
                                ),
                                stopperOff(),
                                new WaitCommand(2000), // Time to fire

                                // 6. Pickup 2 Movement
                                new ParallelCommandGroup(
                                        stopperOn(),
                                        intakeOn(),
                                        transferOn(),
                                        new FollowPathCommand(follower, pickup2).setGlobalMaxPower(0.3)
                                ),

                                // 7. Score 2 Movement/Setup
                                new ParallelCommandGroup(
                                        intakeLow(),
                                        transferLow(),
                                        new FollowPathCommand(follower, score2).setGlobalMaxPower(1)
                                ),

                                // 8. Score 2 Sequence
                                new ParallelCommandGroup(
                                        intakeOn(),
                                        transferOn()
                                ),

                                stopperOff(),
                                new WaitCommand(2000),

                                new ParallelCommandGroup(
                                       stopperOn(),
                                       intakeOn(),
                                       transferOn(),
                                        new FollowPathCommand(follower, pickup3).setGlobalMaxPower(0.3)
                                ),

                                new ParallelCommandGroup(
                                        intakeLow(),
                                        transferLow(),
                                        new FollowPathCommand(follower, score3)
                                ),

                                new ParallelCommandGroup(
                                        intakeOn(),
                                        transferOn()
                                ),

                                stopperOff(),
                                new WaitCommand(2000),

                                new FollowPathCommand(follower, park),

                // ----------------------------------------------------------------------
                // STEP 2: STOP (This interrupts the perpetual commands from Step 1)
                // ----------------------------------------------------------------------
                                new ParallelCommandGroup(
                                        intakeOff(),
                                        stopperOn(),
                                        transferOff(),
                                        turretZero,      // Interrupts turretScore
                                        shooterOff()     // Interrupts shooterOn
                                )
        );



        schedule (new ParallelCommandGroup(
                BlueClose
        ));


    }

    @Override
    public void run(){

        follower.update();
        super.run();


        telemetry.addData("X", follower.getPose().getPose());
        telemetry.addData("intakePower", intakeSubsystem.getCurrentCommand());
        telemetry.addData("turret", turretSubsystem.getCurrentCommand());
        telemetry.update();


    }

}
