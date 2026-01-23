/*
Copyright (c) 2024 Limelight Vision

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.Rev9AxisImuOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

/*
 * This OpMode illustrates how to use the Limelight3A Vision Sensor.
 *
 * @see <a href="https://limelightvision.io/">Limelight</a>
 *
 * Notes on configuration:
 *
 *   The device presents itself, when plugged into a USB port on a Control Hub as an ethernet
 *   interface.  A DHCP server running on the Limelight automatically assigns the Control Hub an
 *   ip address for the new ethernet interface.
 *
 *   Since the Limelight is plugged into a USB port, it will be listed on the top level configuration
 *   activity along with the Control Hub Portal and other USB devices such as webcams.  Typically
 *   serial numbers are displayed below the device's names.  In the case of the Limelight device, the
 *   Control Hub's assigned ip address for that ethernet interface is used as the "serial number".
 *
 *   Tapping the Limelight's name, transitions to a new screen where the user can rename the Limelight
 *   and specify the Limelight's ip address.  Users should take care not to confuse the ip address of
 *   the Limelight itself, which can be configured through the Limelight settings page via a web browser,
 *   and the ip address the Limelight device assigned the Control Hub and which is displayed in small text
 *   below the name of the Limelight on the top level configuration screen.
 */
@Configurable
@Config
//@TeleOp(name = "cameraTesting", group = "Sensor")
public class Cameratesting extends OpMode {

    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx turret;

    private Limelight3A limelight;

    public double lastAngle;
    public IMU imu;
    public YawPitchRollAngles angles;

    private YawPitchRollAngles lastAngles;
    private double globalAngle;


    private double shift = 1;


    private static double turret_tPERd = 4.233;
    private static double angleWant = 0;
    private static double slow = 1;

    private static double p = 0.02;
    private static double i = 0.002;
    private static double d = 0.0005;
    private static double f = 0;

    PIDFController turretController = new PIDFController(p,i,d,f);

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int Limelightheight = 6;

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

//        Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection usb = Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection.LEFT;
//        Rev9AxisImuOrientationOnRobot.LogoFacingDirection logo = Rev9AxisImuOrientationOnRobot.LogoFacingDirection.DOWN;

        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        //Rev9AxisImuOrientationOnRobot orientationOnRobot = new Rev9AxisImuOrientationOnRobot(logo, usb);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        lastAngle = 0;
        limelight.pipelineSwitch(0);



        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

    }

        @Override
        public void start(){
            limelight.start();
        }




        @Override
        public void loop(){

            turretController.setPIDF(p,i,d,f);


            LLResult result = limelight.getLatestResult();

            double tx = result.getTx();



            /** Pipeline 0: yellow detection
             Pipeline 1: blue detection
             Pipeline 2: red detection **/


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




            double facing = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double y = gamepad1.left_stick_y;
            //double y = 0;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            double rotX = x * Math.cos(-facing) - y * Math.sin(-facing);
            rotX = rotX * 1.1;
            double rotY = x * Math.sin(-facing) + y * Math.cos(-facing);

            double d = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(rx), 1);

            double lf = (rotY + rotX + rx) / d;
            double lb = (rotY - rotX + rx) / d;
            double rf = (rotY - rotX - rx) / d;
            double rb = (rotY + rotX - rx) / d;


            leftBack.setVelocity(3000 * lb * shift);
            leftFront.setVelocity(3000 * lf * shift);
            rightBack.setVelocity(3000 * rb * shift);
            rightFront.setVelocity(3000 * rf * shift);


            if (gamepad1.back) {
                imu.resetYaw();
            }




            double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            if (gamepad1.right_trigger > 0.1) {imu.resetYaw();}
            double turretAngle = turret.getCurrentPosition()/turret_tPERd;
            double target = normA(angleWant - robotHeading - tx);
            if (target > 135) {target = 135;} else if (target < -135) {target = -135;}
            double error = target - turretAngle;
            double turretPower = clamp(error * slow, -1, 1);
//            turret.setPower(turretController.calculate(turretAngle, target));
            turret.setVelocity(turretController.calculate(turretAngle, target) * 1400 - imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate * turret_tPERd);

            //turret.setVelocity(-imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate * turret_tPERd * slow);


            telemetry.addData("turretPos", "%d", turret.getCurrentPosition());
            telemetry.addData("robotHeading", "%f", robotHeading);
            telemetry.addData("turretAngle", "%f", turretAngle);
            telemetry.addData("turretTarget", "%f", target);
            telemetry.addData("error", "%f", error);
            telemetry.addData("turretPower", "%f", turret.getVelocity());
            telemetry.addData("tx", tx);
            telemetry.update();
        }

        @Override
        public void stop(){
        limelight.stop();
        }

    public double normA(double angle) {angle %= 360; if (angle < -180) angle += 360; else if (angle > 180) angle -= 360;return angle;}
    public double clamp(double x, double min, double max) {return Math.max(min,Math.min(max,x));}
    }


