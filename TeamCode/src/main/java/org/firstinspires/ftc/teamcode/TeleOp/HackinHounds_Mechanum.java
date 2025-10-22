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
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Hardware.HackinHoundsHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;
import com.bylazar.configurables.annotations.Configurable;

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
 */

@Configurable
@TeleOp(name="Mechanum", group="Linear OpMode")
public class HackinHounds_Mechanum extends OpMode {
    // Declare OpMode members. aamir dont screw stuff up
    private ElapsedTime runtime = new ElapsedTime();
    private HackinHoundsHardware robot = new HackinHoundsHardware();


    double shift = 1;

//    public static double shooterPower = 0;


    double cycleStart = 0;

    private Limelight3A limelight;

    private static double Kp = 3;
    private static double Ki = 0;
    private static double Kd = 0.05;


    private double currentVoltage;

    double integralSum = 0;
    double lastError = 0;
    ElapsedTime timer = new ElapsedTime();


    boolean targetVisible = false;

    private static double tolerance = 2.0;
    private static double desiredOffset = 0;

    private static double horizontalOffset;


    @Override
    public void init() {
        robot.init(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
//skibidi
        telemetry.addData("Status", "Initialized");
        telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.update();
    }


    public static double shooterPower = 0;

    double turnPower = 0;
    @Override
    public void start(){
        limelight.start();
        runtime.reset();

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



            YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw());

            LLResult llResult = limelight.getLatestResult();
            if (llResult != null && llResult.isValid()){
                Pose3D botPose = llResult.getBotpose_MT2();
                telemetry.addData("Tx", llResult.getTx());
                telemetry.addData("Ty", llResult.getTy());
                telemetry.addData("Ta", llResult.getTa());
                telemetry.addData("Botpose", llResult.getBotpose_MT2());
            }

            LLResult result = limelight.getLatestResult();

            /** limelight button to auto align robot **/
            if (result.isValid()) {
                // Read tx (in degrees)
                double tx = result.getTx();

// Get the robot's current heading from IMU (in radians)
                double currentAngle = robot.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

// Compute the desired *absolute* angle in IMU space
                double targetAngle = currentAngle + Math.toRadians(tx);
// PID error becomes how far the robot is from that new target angle
                turnPower = PIDControl(targetAngle, currentAngle);

            }
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


            if (gamepad2.a) {
                robot.leftFront.setPower(turnPower);
                robot.leftBack.setPower(turnPower);
                robot.rightBack.setPower(-turnPower);
                robot.rightFront.setPower(-turnPower);
            } else {
                robot.leftBack.setVelocity(3000 * lb * shift);
                robot.leftFront.setVelocity(3000 * lf * shift);
                robot.rightBack.setVelocity(3000 * rb * shift);
                robot.rightFront.setVelocity(3000 * rf * shift);
            }

            if (gamepad1.back) {
                robot.imu.resetYaw();
            }

            if (gamepad1.a) {
                robot.intake.setPosition(0);
            }

                /** intake code prototype **/


                //telemetry.addData("Driving Finished", "%f", runtime.milliseconds() - cycleStart);


            telemetry.addData("Telemtry finished", "%f", runtime.milliseconds() - cycleStart);
            telemetry.addData("IMU HEADING:", "%f", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("turnPower", turnPower);
//        telemetry.addData("rightBack", "%f", robot.rightBack.getVelocity());
//        telemetry.addData("leftFront", "%f", robot.leftFront.getPower());
//        telemetry.addData("leftBack", "%f", robot.leftBack.getPower());
//        telemetry.addData("rightFront", "%f", robot.rightFront.getPower());
            telemetry.addData("currentVoltage", "%f", currentVoltage);

            telemetry.update();

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


    double angleWrap ( double radians){
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }
}




