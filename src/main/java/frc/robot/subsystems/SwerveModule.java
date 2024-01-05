package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ModuleConstants;
import frc.robot.utils.Enums.ModulePosition;

public class SwerveModule extends SubsystemBase {
    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_turningMotor;
    private final RelativeEncoder m_driveEncoder;
    private final CANCoder m_turnCANcoder;
    private final RelativeEncoder m_turningEncoder;
    private final double m_turningEncoderOffset;

    public SwerveModule(
            ModulePosition modulePosition,
            int driveMotorCanChannel,
            int turningMotorCanChannel,
            int cancoderCanChannel,
            boolean driveMotorReversed,
            boolean turningMotorReversed,
            double turningEncoderOffset) {
        // Drive Motor setup
        m_driveMotor = new CANSparkMax(driveMotorCanChannel, MotorType.kBrushless);
        m_driveMotor.restoreFactoryDefaults();
        m_driveMotor.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
        m_driveMotor.enableVoltageCompensation(ModuleConstants.kVoltCompensation);
        m_driveMotor.setInverted(driveMotorReversed);
        m_driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // turning motor setup
        m_turningMotor = new CANSparkMax(turningMotorCanChannel, MotorType.kBrushless);
        m_turningMotor.restoreFactoryDefaults();
        m_turningMotor.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);
        m_turningMotor.enableVoltageCompensation(ModuleConstants.kVoltCompensation);
        m_turningMotor.setInverted(turningMotorReversed);
        m_turningMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // drive encoder setup
        m_driveEncoder = m_driveMotor.getEncoder();
        m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveMetersPerEncRev);
        m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kModuleFreeSpeedMetersPerSec);

        // turning encoder setup
        // absolute encoder used to establish known wheel position on start position
        m_turnCANcoder = new CANCoder(cancoderCanChannel);
        m_turnCANcoder.configFactoryDefault();
        m_turnCANcoder.configAllSettings(GenerateCanCoderConfig());
        m_turningEncoder = m_turningMotor.getEncoder();
        m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurnDegreesPerEncRev);
        m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurnDegreesPerEncRev / 60);
        m_turningEncoderOffset = turningEncoderOffset;

    }

    private static CANCoderConfiguration GenerateCanCoderConfig() {
        CANCoderConfiguration sensorConfig = new CANCoderConfiguration();

        sensorConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        sensorConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        sensorConfig.sensorTimeBase = SensorTimeBase.PerSecond;

        return sensorConfig;
    }
}
