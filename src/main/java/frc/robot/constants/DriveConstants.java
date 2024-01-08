package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public final class DriveConstants {
    public static final boolean kFrontLeftTurningMotorReversed = true;
    public static final boolean kFrontRightTurningMotorReversed = true;
    public static final boolean kRearLeftTurnMotorReversed = true;
    public static final boolean kRearRightTurnMotorReversed = true;

    public static final boolean kFrontLeftDriveMotorReversed = true;
    public static final boolean kFrontRightDriveMotorReversed = true;
    public static final boolean kRearLeftDriveMotorReversed = true;
    public static final boolean kRearRightDriveMotorReversed = true;

    public static final double kFrontLeftAngularOffset = 41.3;
    public static final double kFrontRightAngularOffset = 113.2;
    public static final double kRearLeftAngularOffset = 125.59;
    public static final double kRearRightAngularOffset = 255.14;

    // Spark MAX Drive Motor CAN IDs
    public static final int kFrontLeftDriveMotorCanId = 23;
    public static final int kFrontRightDriveMotorCanId = 22;
    public static final int kRearLeftDriveMotorCanId = 26;
    public static final int kRearRightDriveMotorCanId = 27;

    // Spark MAX Turn Motor CAN IDs
    public static final int kFrontLeftTurnMotorCanId = 21;
    public static final int kFrontRightTurnMotorCanId = 20;
    public static final int kRearLeftTurnMotorCanId = 24;
    public static final int kRearRightTurnMotorCanId = 25;

    // Spark MAX Turn Encoder CAN IDs
    public static final int kFrontLeftTurnEncoderCanId = 50;
    public static final int kFrontRightTurnEncoderCanId = 51;
    public static final int kRearLeftTurnEncoderCanId = 53;
    public static final int kRearRightTurnEncoderCanId = 52;

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxRotationRadiansPerSecond = Math.PI;


    // Chassis configuration
    public static final double kRobotWidth = Units.inchesToMeters(23.25);  // Distance between centers of right and left wheels on robot
    public static final double kRobotLength = Units.inchesToMeters(19.25); // Distance between front and back wheels on robot

    // Drive Control
    public static final boolean kFieldRelative = true;
    public static final double kTranslationSlew = 1.55;
    public static final double kRotationSlew = 3.00;
}
