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

package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.control.feedback.PIDController;

import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDFController;

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

@Config
@TeleOp(name="ErikIsBalding", group="Linear OpMode")
//@Disabled
public class ErikThing extends LinearOpMode {

    //    public DcMotorEx leftFront;
//    public DcMotorEx rightFront;
//    public DcMotorEx leftBack;
//    public DcMotorEx rightBack;
    public DcMotorEx left;
    public Servo hoodAngler;
    int shift = 1;

    public static double leftPower = 0;


    public static double targetVelocity = 0;


    public double ticksPerRevolution = 28;
    public double currentVelocity;
    public double rpm;

    public static double P = 11;
    public static double I = 0;
    public static double D = 0.00;
    public static double F = 0.8;

    public static double hoodPosition = 0.00;


    private static PIDFController shooterController = new PIDFController(P, I, D, F);

    @Override
    public void runOpMode() {

        left  = hardwareMap.get(DcMotorEx.class, "left");
//        right = hardwareMap.get(DcMotorEx.class, "right");
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        shift = 0;




        waitForStart();
        while (opModeIsActive()) {
//            if (gamepad1.dpad_up) {
//                right.setPower(rightPower);
//                left.setPower(leftPower);
//            }
            shooterController.setPIDF(P,I,D,F);


            double output = shooterController.calculate(
                    left.getVelocity(), targetVelocity
            );
            left.setVelocity(output);




//            while (!pController.atSetPoint()) {
//                double output = pController.calculate(
//                        left.getVelocity()  // the measured value
//                );
//                left.setVelocity(output);
//            }
//            left.setPower(0);

//            left.setPower(leftPower);

            if (gamepad1.x) {
                shift = 0;
            } else if (gamepad1.y) {
                shift = 1;
            } else if (gamepad1.b) {
                shift = 2;
            } else if (gamepad1.a) {
                shift = 3;
            }


            currentVelocity = left.getVelocity();

            rpm = currentVelocity/ticksPerRevolution * 60;



            telemetry.addData("leftpower", "%f", left.getPower());
            telemetry.addData("leftRPM", "%f", left.getVelocity());
            telemetry.addData("RPM", rpm);
            telemetry.update();

            FtcDashboard dashboard = FtcDashboard.getInstance();
            Telemetry dashboardTelemetry = dashboard.getTelemetry();

            dashboardTelemetry.addData("rpm", rpm );
            dashboardTelemetry.addData("targetVelocity", targetVelocity);
            dashboardTelemetry.addData("currentVelocity", left.getVelocity());
            dashboardTelemetry.addData("output", output);
            dashboardTelemetry.update();

        }
    }
}
