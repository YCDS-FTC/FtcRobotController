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

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

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
@Configurable
@TeleOp(name="ColemanIsBalding", group="Linear OpMode")
//@Disabled
public class ColemanThing extends LinearOpMode {
    private PanelsTelemetry panelsTelemetry;

//
//    public DcMotorEx left;
//    public DcMotorEx right;
    public static double shift = 0;
    public Servo flip;
    public Servo intake;
    public AnalogInput test1;
    public AnalogInput test2;
    public AnalogInput test3;

    @Override
    public void runOpMode() {
//        left  = hardwareMap.get(DcMotorEx.class, "left");
//        right = hardwareMap.get(DcMotorEx.class, "right");
//        left.setDirection(DcMotorSimple.Direction.REVERSE);
//        right.setDirection(DcMotorSimple.Direction.FORWARD);
//        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        flip = hardwareMap.get(Servo.class, "flip");
        intake = hardwareMap.get(Servo.class, "intake");
//        rspin = hardwareMap.get(Servo.class, "rspin");
//        lspin = hardwareMap.get(Servo.class, "lspin");
        test1 = hardwareMap.get(AnalogInput.class, "test1");
        test2 = hardwareMap.get(AnalogInput.class, "test2");
        test3 = hardwareMap.get(AnalogInput.class, "test3");
        shift = 0;



        PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;

        waitForStart();
        while (opModeIsActive()) {


            if (getDistance(test1) < 7 && getDistance(test2) < 7 && getDistance(test3) < 7) {
                telemetry.addLine("Thingy is filled fyi");
                intake.setPosition(0.5);
            } else if (getDistance(test2) < 7 && getDistance(test3) < 7) {
                telemetry.addLine("Lets continue");
                intake.setPosition(0);
            }

            telemetry.addData("", "%f", getDistance(test1));
            telemetry.addData("", "%f", getDistance(test2));
            telemetry.addData("", "%f", getDistance(test3));

//            if (gamepad1.a) {
//                flip.setPosition(0.53);
//            } else {
//                flip.setPosition(0.49);
//            }
            //intake.setPosition(shift);
            // 0.49 to 0.53

            if (gamepad1.dpad_up) {
                intake.setPosition(1);
            } else if (gamepad1.dpad_down) {
                intake.setPosition(0);
            } else if (gamepad1.dpad_left) {
                intake.setPosition(0.5);
            }

            telemetry.update();
        }
    }
    public double getDistance (AnalogInput sensor) {
        return (sensor.getVoltage() * 32.50930976) - 2.6953842;
    }
}
