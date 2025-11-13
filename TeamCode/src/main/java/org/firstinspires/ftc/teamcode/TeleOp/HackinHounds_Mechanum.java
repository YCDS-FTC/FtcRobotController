/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Size;

import com.bylazar.field.PanelsField;
import com.bylazar.panels.PanelsConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Hardware.HackinHoundsHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;
import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Hardware.Drawing;
import java.util.ArrayList;
import java.util.List;



/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 *
 *
 * Greetings, Coleman Malcarne Was Here, and if youre seeing this then why are yall still using my old code, make youre own smh
 */
@Config
@Configurable
@TeleOp(name="Mechanum", group="Linear OpMode")
public class HackinHounds_Mechanum extends OpMode {
    // Declare OpMode members. aamir dont screw stuff up
    private ElapsedTime runtime = new ElapsedTime();
    private HackinHoundsHardware robot = new HackinHoundsHardware();

    double shift = 1;
    double ErikFlicksUp;

//    public static double shooterPower = 0;


    double cycleStart = 0;

    public static double Kp = 1;
    public static double Ki = 0;
    public static double Kd = 0.05;


    private double currentVoltage;

    double integralSum = 0;
    double lastError = 0;
    ElapsedTime timer = new ElapsedTime();


    boolean targetVisible = false;

    private static double tolerance = 2.0;
    private static double desiredOffset = 0;

    private static double horizontalOffset;

    boolean wasDetecting = false;


    private final ElapsedTime sensorTimer = new ElapsedTime();

    public static int flickTargetPosition = 0;


    @Override
    public void init() {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.update();
        robot.limelight.pipelineSwitch(0);

    }



    double turnPower = 0;
    @Override
    public void start(){
        runtime.reset();
        robot.limelight.start();
        robot.flick.setTargetPosition(0);
        robot.flick.setPower(0);
    }



    @Override
    public void loop () {
            currentVoltage = robot.voltageSensor.getVoltage();

            cycleStart = runtime.milliseconds();
            //telemetry.addData("Begin Time", "%f", runtime.milliseconds() - cycleStart);

            if (gamepad1.dpad_up) {
                shift = 1;
            } else if (gamepad1.dpad_right) {
                shift = 0.75;
            } else if (gamepad1.dpad_down) {
                shift = 0.5;
            }
            if (gamepad1.left_stick_button) {
                shift = 1;
            }


//
            YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
            robot.limelight.updateRobotOrientation(orientation.getYaw());
            LLResult llResult = robot.limelight.getLatestResult();
            if (llResult != null && llResult.isValid()){
                Pose3D botPose = llResult.getBotpose_MT2();
                telemetry.addData("Tx", llResult.getTx());
                telemetry.addData("Ty", llResult.getTy());
                telemetry.addData("Ta", llResult.getTa());
                telemetry.addData("Botpose", llResult.getBotpose_MT2());
            }

            LLResult result = robot.limelight.getLatestResult();

            double ty = result.getTy();
            double tx = result.getTx();

        /** limelight button to auto align robot **/
            if (result.isValid()) {
//                robot.getshooterPower();
                // Read tx (in degrees)

//// Get the robot's current heading from IMU (in radians)
                double currentAngle = robot.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
//
//// Compute the desired *absolute* angle in IMU space
                double targetAngle = currentAngle + Math.toRadians(tx);
//// PID error becomes how far the robot is from that new target angle
                turnPower = PIDControl(targetAngle, currentAngle);
//                double angleError = targetAngle - currentAngle;
//                double turnSign = Math.signum(targetAngle - robot.getAngle());
//                turnPower = Math.min(Math.abs((turnSign*angleError)/45.0), 0.3);

            }


        double distanceToGoal =  robot.limelight(ty, tx);
        double motorPower = robot.getshooterPower(distanceToGoal);
        double hoodAngle = robot.getHoodAngle(distanceToGoal);


                // Send data to Dashboard
            TelemetryPacket packet = new TelemetryPacket();




                /** DRIVING - Perfectly FINE (don't change) **/

                //telemetry.addData("begining if's finished", "%f", runtime.milliseconds() - cycleStart);

            double facing = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double y = -gamepad1.left_stick_y;
            //double y = 0;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double rotX = x * Math.cos(-facing) - y * Math.sin(-facing);
            rotX = rotX * 1.1;
            double rotY = x * Math.sin(-facing) + y * Math.cos(-facing);

            double d = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(rx), 1);

            double lf = (rotY + rotX + rx) / d;
            double lb = (rotY - rotX + rx) / d;
            double rf = (rotY - rotX - rx) / d;
            double rb = (rotY + rotX - rx) / d;

            if (gamepad2.dpad_up){
                robot.shooterMotor.setVelocity(0);
            }


            if (gamepad2.y) {
                robot.leftFront.setPower(0.5 * lb * shift + turnPower * 0.5);
                robot.leftBack.setPower(0.5 * lb * shift + turnPower * 0.5);
                robot.rightBack.setPower(0.5 * lb * shift - turnPower * 0.5);
                robot.rightFront.setPower(0.5 * lb * shift - turnPower * 0.5);

                robot.getshooterPower(distanceToGoal);
                robot.getHoodAngle(distanceToGoal);
                robot.angleServo.setPosition(hoodAngle);
                robot.shooterMotor.setVelocity(motorPower);

            } else {
                robot.leftBack.setVelocity(3000 * lb * shift);
                robot.leftFront.setVelocity(3000 * lf * shift);
                robot.rightBack.setVelocity(3000 * rb * shift);
                robot.rightFront.setVelocity(3000 * rf * shift);
            }

            if (gamepad1.back) {
                robot.imu.resetYaw();
            }


        /** intake code prototype **/

            if(gamepad2.x){
                robot.intake.setPosition(0.5);
                robot.intake2.setPosition(0.5);
            }
            if (gamepad2.a) {
                robot.intake.setPosition(0);
                robot.intake2.setPosition(1);
            }
            if (gamepad2.b){
                robot.intake.setPosition(1);
                robot.intake2.setPosition(0);
            }

            double d1 = robot.getDistance(robot.test1), d2 = robot.getDistance(robot.test2), d3 = robot.getDistance(robot.test3);
            if (d1 < 8 && d2 < 7.5 && d3 < 7){
                if(!wasDetecting){
                    wasDetecting = true;
                    sensorTimer.reset();
                }
                if (sensorTimer.seconds() > 0.1){
                    telemetry.addLine("Thingy is filled fyi");
                    robot.intake.setPosition(0.5);
                    robot.intake2.setPosition(0.5);
                    robot.light.setPosition(0.6);
                    robot.light2.setPosition(0.6);
                }
                telemetry.addData("confirming secs", "%.3f", sensorTimer.seconds());
            }  else if (d1 < 7.5 && d2 < 7) {
                wasDetecting = false;
                telemetry.addLine("Lets continue");
                robot.light.setPosition(0);
                robot.light2.setPosition(0);
                robot.intake.setPosition(0);
                robot.intake2.setPosition(1);
            } else{
                wasDetecting = false;
                robot.light.setPosition(0);
                robot.light2.setPosition(0);
                sensorTimer.reset();
            }

            if (gamepad2.left_bumper) {
                robot.flick.setTargetPosition(0);
            }

            if (gamepad2.right_bumper) {
                ErikFlicksUp = timer.seconds();
                robot.flick.setTargetPosition(50);
                robot.flick.setPower(1);
            } else if (robot.flick.getCurrentPosition() >= 49 || timer.seconds() - ErikFlicksUp > 1.2) {
                robot.flick.setTargetPosition(0);
            } else if (robot.flick.getCurrentPosition() <= 1) {
                robot.flick.setPower(0.1);
            }
//
//


                //telemetry.addData("Driving Finished", "%f", runtime.milliseconds() - cycleStart);


            telemetry.addData("Telemtry finished", "%f", runtime.milliseconds() - cycleStart);
            telemetry.addData("IMU HEADING:", "%f", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("turnPower", turnPower);
            telemetry.addData("currentVoltage", "%f", currentVoltage);

            telemetry.addData("distanceToGoal", "%f",distanceToGoal);
            telemetry.addData("shooterPower", "%f", motorPower);

            telemetry.addData("", "%f", robot.getDistance(robot.test1));
            telemetry.addData("", "%f", robot.getDistance(robot.test2));
            telemetry.addData("", "%f", robot.getDistance(robot.test3));

            telemetry.addData("velocity", "%f", robot.leftBack.getVelocity());
            telemetry.addData("flickTargetPos", "%d", robot.flick.getTargetPosition());
            telemetry.addData("flickPos", "%d", robot.flick.getCurrentPosition());

            telemetry.addData("leftBack", "%f", robot.leftBack.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("leftFront", "%f", robot.leftFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("rightBack", "%f", robot.rightBack.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("rightFront", "%f", robot.rightFront.getCurrent(CurrentUnit.AMPS));


        telemetry.update();


            Pose2D pose2d = robot.pinpoint.getPosition();
            Pose pose = new Pose(pose2d.getX(DistanceUnit.INCH), pose2d.getY(DistanceUnit.INCH), pose2d.getHeading(AngleUnit.RADIANS));
            Drawing.drawRobot(pose);
            Drawing.sendPacket();
    }


    double PIDControl(double reference, double state){
        double error = angleWrap(reference - state);

        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        timer.reset();

        double output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
        return output;
    }


    double angleWrap (double radians){
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }
}




