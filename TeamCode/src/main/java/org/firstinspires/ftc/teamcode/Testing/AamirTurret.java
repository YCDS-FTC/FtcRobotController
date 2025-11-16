//package org.firstinspires.ftc.teamcode.Testing;
//
//import static androidx.core.math.MathUtils.clamp;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.bylazar.configurables.annotations.Configurable;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//
//import kotlin.properties.ObservableProperty;
//
//
//
//@Configurable
//@Config
//@TeleOp(name = "aamirTurret", group = "TeleOp")
//public class AamirTurret extends OpMode {
//
//
//    public static double Kp = 0;
//    public static double Ki = 0;
//    public static double Kd = 0;
//    public static double Kf = 0;
//
//    private static final int TICKS_PER_MOTOR_REV = 538;// change to your motor's encoder CPR
//    private static final int Gear_Ratio = 4;
//    private static final double Center_Angle = 0;
//
//    private static final double Min_Angle = 0;
//    private static final double Max_Ange = 360;
//    private static final double Max_Power = 0.35;
//
//    private static final double Deg_Acceptable = 1;
//
//
//    double integralSum = 0;
//    double lastError = 0;
//    ElapsedTime timer = new ElapsedTime();
//    double turnPower = 0;
//
//
//    private Limelight3A limelight;
//    private IMU imu;
//    private DcMotorEx turretMotor;
//
//    @Override
//    public void init(){
//
//        limelight.pipelineSwitch(0);
//
//    }
//
//
//    @Override
//    public void start(){
//        limelight.start();
//        resetRuntime();
//    }
//
//
//    @Override
//    public void loop (){
//
//
//        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
//        limelight.updateRobotOrientation(orientation.getYaw());
//        LLResult llResult = limelight.getLatestResult();
//        if (llResult != null && llResult.isValid()){
//            Pose3D botPose = llResult.getBotpose_MT2();
//            telemetry.addData("Tx", llResult.getTx());
//            telemetry.addData("Ty", llResult.getTy());
//            telemetry.addData("Ta", llResult.getTa());
//            telemetry.addData("Botpose", llResult.getBotpose_MT2());
//        }
//
//        LLResult result = limelight.getLatestResult();
//
//        double ty = result.getTy();
//        double tx = result.getTx();
//
//
//
//        if (result.isValid()) {
////                robot.getshooterPower();
//            // Read tx (in degrees)
//
////// Get the robot's current heading from IMU (in radians)
//            double currentAngle = ticksToAngle(turretMotor.getCurrentPosition());
//            double desiredAngleDeg = currentAngle + tx; // or -tx depending on direction
//            desiredAngleDeg = clamp(desiredAngleDeg, Min_Angle, Max_Ange);
//
////// Compute the desired *absolute* angle in IMU space
//            double targetAngle = currentAngle + Math.toRadians(tx);
////// PID error becomes how far the robot is from that new target angle
//            turnPower = PIDControl(targetAngle, currentAngle);
////                double angleError = targetAngle - currentAngle;
////                double turnSign = Math.signum(targetAngle - robot.getAngle());
////                turnPower = Math.min(Math.abs((turnSign*angleError)/45.0), 0.3);
//
//        }
//    }
//
//    @Override
//    public void stop(){
//        limelight.stop();
//
//    }
//
//    double PIDControl(double reference, double state){
//        double error = angleWrap(reference - state);
//
//        integralSum += error * timer.seconds();
//        double derivative = (error - lastError) / timer.seconds();
//        lastError = error;
//        timer.reset();
//
//        double output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
//        return output;
//    }
//
//
//    double angleWrap (double radians){
//        while (radians > Math.PI) radians -= 2 * Math.PI;
//        while (radians < -Math.PI) radians += 2 * Math.PI;
//        return radians;
//    }
//
//    double ticksToAngle (int motorTicks){
//        return (double) motorTicks * 360/ TICKS_PER_MOTOR_REV * Gear_Ratio;
//    }
//}
