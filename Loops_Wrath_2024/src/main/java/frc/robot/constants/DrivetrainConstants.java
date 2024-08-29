package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class DrivetrainConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 5.2;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 4.8; // radians per second
    public static final double kMagnitudeSlewRate = 6.4; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 8.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(28-1.75*2);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(28-1.75*2);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    
    public static final double kFrontLeftChassisAngularOffset = 0.0;// Math.PI
    public static final double kFrontRightChassisAngularOffset = Math.PI / 2;//-Math.PI / 2;
    public static final double kBackLeftChassisAngularOffset = -Math.PI/2;//Math.PI / 2;
    public static final double kBackRightChassisAngularOffset = Math.PI; //0;

    /*
    public static final double kFrontLeftChassisAngularOffset = -2*Math.PI;
    public static final double kFrontRightChassisAngularOffset = -2*Math.PI;
    public static final double kBackLeftChassisAngularOffset = -2*Math.PI;
    public static final double kBackRightChassisAngularOffset = -2*Math.PI-(4/180.0);
    */

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;//8
    public static final int kRearLeftDrivingCanId = 6;//3
    public static final int kFrontRightDrivingCanId = 3;//6
    public static final int kRearRightDrivingCanId = 8;//1

    public static final int kFrontLeftTurningCanId = 5;//4
    public static final int kRearLeftTurningCanId = 2;//7
    public static final int kFrontRightTurningCanId = 7;//2
    public static final int kRearRightTurningCanId = 4;//5

    public static final double kTurnToleranceDeg = 0;
    public static final double kTurnRateToleranceDegPerS = 0;

    public static final boolean kGyroReversed = true;

    public static final double kRobotStartOffset = 0;
}
