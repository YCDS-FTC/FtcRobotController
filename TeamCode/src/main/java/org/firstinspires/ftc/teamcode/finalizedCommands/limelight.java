//package org.firstinspires.ftc.teamcode.finalizedCommands;
//
//import com.seattlesolvers.solverslib.util.InterpLUT;
//
//public class limelight {
//
//    InterpLUT shooterPower = new InterpLUT();
//
//
//    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
//    NetworkTableEntry ty = table.getEntry("ty");
//    double targetOffsetAngle_Vertical = ty.getDouble(0.0);
//
//    // how many degrees back isA your limelight rotated from perfectly vertical?
//    private double limelightMountAngleDegrees = 25.0;
//
//    // distance from the center of the Limelight lens to the floor
//    private double limelightLensHeightInches = 20.0;
//
//    // distance from the target to the floor
//    private double goalHeightInches = 30.0;
//
//    private double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
//    private double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
//
//    //calculate distance
//
//
//    public double limelightGetDistance(){
//        double distancetoGoal = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
//
//        return distancetoGoal;
//    }
//
//    public double Interpolation(){
//
//
//
//        shooter
//    }
//
//}
