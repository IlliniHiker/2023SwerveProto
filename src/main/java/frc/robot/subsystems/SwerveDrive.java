package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants;
import frc.robot.utils.Enums.ModulePosition;

public class SwerveDrive extends SubsystemBase {
    private final SwerveModule m_frontLeft = new SwerveModule(
            ModulePosition.FRONT_LEFT,
            DriveConstants.kFrontLeftDriveMotorCanId,
            DriveConstants.kFrontLeftTurnMotorCanId,
            DriveConstants.kFrontLeftTurnEncoderCanId,
            DriveConstants.kFrontLeftDriveMotorReversed,
            DriveConstants.kFrontLeftTurningMotorReversed,
            DriveConstants.kFrontLeftAngularOffset);

    private final SwerveModule m_frontRight = new SwerveModule(
            ModulePosition.FRONT_RIGHT,
            DriveConstants.kFrontRightDriveMotorCanId,
            DriveConstants.kFrontRightTurnMotorCanId,
            DriveConstants.kFrontRightTurnEncoderCanId,
            DriveConstants.kFrontRightDriveMotorReversed,
            DriveConstants.kFrontRightTurningMotorReversed,
            DriveConstants.kFrontRightAngularOffset);

    private final SwerveModule m_rearLeft = new SwerveModule(
            ModulePosition.REAR_LEFT,
            DriveConstants.kRearLeftDriveMotorCanId,
            DriveConstants.kRearLeftTurnMotorCanId,
            DriveConstants.kRearLeftTurnEncoderCanId,
            DriveConstants.kRearLeftDriveMotorReversed,
            DriveConstants.kRearLeftTurnMotorReversed,
            DriveConstants.kRearLeftAngularOffset);

    private final SwerveModule m_rearRight = new SwerveModule(
            ModulePosition.REAR_RIGHT,
            DriveConstants.kRearRightDriveMotorCanId,
            DriveConstants.kRearRightTurnMotorCanId,
            DriveConstants.kRearRightTurnEncoderCanId,
            DriveConstants.kRearRightDriveMotorReversed,
            DriveConstants.kRearRightTurnMotorReversed,
            DriveConstants.kRearRightAngularOffset);

    private final AHRS m_gyro;

    public SwerveDrive() {
        m_gyro = new AHRS(SPI.Port.kMXP);

    }

    public double getAngle() {
        return m_gyro.getAngle();
    }

    public double getYaw() {
        return m_gyro.getYaw();
    }

    public double getRoll() {
        return m_gyro.getRoll();
    }

    public double getPitch() {
        return m_gyro.getPitch();
    }
}
