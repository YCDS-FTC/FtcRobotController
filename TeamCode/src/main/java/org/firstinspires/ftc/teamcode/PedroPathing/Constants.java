
package org.firstinspires.ftc.teamcode.PedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14.42424)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .forwardZeroPowerAcceleration(-30.31722854053854)
            .lateralZeroPowerAcceleration(-77.33305472080436)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.18, 0, 0.018, 0))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.25, 0, 0.02, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(1.2, 0, 0.06, 0))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1,0.0001,0.03,0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.015,0.0,0.00004,0.6,0.0))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.001,0,0.00001,0, 0))
            .centripetalScaling(0.001);

    //10.3

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(82.4749395415539)
            .yVelocity(63.93339574618602);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(3.61807087)
            .strafePodX(-3.4649606)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}

