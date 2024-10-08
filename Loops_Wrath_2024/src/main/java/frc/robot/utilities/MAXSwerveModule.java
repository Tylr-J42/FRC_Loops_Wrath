package frc.robot.utilities;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;

import frc.robot.constants.SwerveModuleConstants;

public class MAXSwerveModule {
	private final CANSparkMax m_turningSparkMax;

	private final AbsoluteEncoder m_turningEncoder;

	private final SparkPIDController m_turningPIDController;

	private final TalonFX m_drivingTalonFX;

	private final VelocityVoltage driveRequest = new VelocityVoltage(0).withSlot(0);

	private double m_chassisAngularOffset = 0;
	private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

	/**
	 * Constructs a MAXSwerveModule and configures the driving and turning motor,
	 * encoder, and PID controller. This configuration is specific to the REV
	 * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
	 * Encoder.
	 */
	public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
		m_drivingTalonFX = new TalonFX(drivingCANId);

		FeedbackConfigs driveFeedConfig = new FeedbackConfigs(); 
		driveFeedConfig.SensorToMechanismRatio = SwerveModuleConstants.kDrivingMotorReduction;

		CurrentLimitsConfigs driveCurrentLimitConfig = new CurrentLimitsConfigs();
		driveCurrentLimitConfig.StatorCurrentLimitEnable = true;
		driveCurrentLimitConfig.StatorCurrentLimit = SwerveModuleConstants.kDrivingStatorCurrentLimit;

		TalonFXConfigurator drivingConfigurator = m_drivingTalonFX.getConfigurator();
		MotorOutputConfigs drivingMotorConfig = new MotorOutputConfigs();
		drivingMotorConfig.Inverted = InvertedValue.Clockwise_Positive;
		drivingMotorConfig.NeutralMode = NeutralModeValue.Brake;

		Slot0Configs driveSlot0Config = new Slot0Configs();
		driveSlot0Config.kP = SwerveModuleConstants.kDrivingP;
		driveSlot0Config.kI = SwerveModuleConstants.kDrivingI;
		driveSlot0Config.kD = SwerveModuleConstants.kDrivingD;
		driveSlot0Config.kS = 0;
		driveSlot0Config.kV = SwerveModuleConstants.kDrivingFF;
		driveSlot0Config.kG = 0;

		drivingConfigurator.apply(driveSlot0Config);
		drivingConfigurator.apply(driveFeedConfig);
		drivingConfigurator.apply(driveCurrentLimitConfig);

		m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

		// Factory reset, so we get the SPARKS MAX to a known state before configuring
		// them. This is useful in case a SPARK MAX is swapped out.
		m_turningSparkMax.restoreFactoryDefaults();

		// Setup encoders and PID controllers for the driving and turning SPARKS MAX.
		m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
		m_turningPIDController = m_turningSparkMax.getPIDController();
		m_turningPIDController.setFeedbackDevice(m_turningEncoder);

		// Apply position and velocity conversion factors for the turning encoder. We
		// want these in radians and radians per second to use with WPILib's swerve
		// APIs.
		m_turningEncoder.setPositionConversionFactor(SwerveModuleConstants.kTurningEncoderPositionFactor);
		m_turningEncoder.setVelocityConversionFactor(SwerveModuleConstants.kTurningEncoderVelocityFactor);

		// Invert the turning encoder, since the output shaft rotates in the opposite direction of
		// the steering motor in the MAXSwerve Module.
		m_turningEncoder.setInverted(SwerveModuleConstants.kTurningEncoderInverted);

		// Enable PID wrap around for the turning motor. This will allow the PID
		// controller to go through 0 to get to the setpoint i.e. going from 350 degrees
		// to 10 degrees will go through 0 rather than the other direction which is a
		// longer route.
		m_turningPIDController.setPositionPIDWrappingEnabled(true);
		m_turningPIDController.setPositionPIDWrappingMinInput(SwerveModuleConstants.kTurningEncoderPositionPIDMinInput);
		m_turningPIDController.setPositionPIDWrappingMaxInput(SwerveModuleConstants.kTurningEncoderPositionPIDMaxInput);

		// Set the PID gains for the turning motor. Note these are example gains, and you
		// may need to tune them for your own robot!
		m_turningPIDController.setP(SwerveModuleConstants.kTurningP);
		m_turningPIDController.setI(SwerveModuleConstants.kTurningI);
		m_turningPIDController.setD(SwerveModuleConstants.kTurningD);
		m_turningPIDController.setFF(SwerveModuleConstants.kTurningFF);
		m_turningPIDController.setOutputRange(SwerveModuleConstants.kTurningMinOutput,
			SwerveModuleConstants.kTurningMaxOutput);

		m_turningSparkMax.setIdleMode(SwerveModuleConstants.kTurningMotorIdleMode);
		m_turningSparkMax.setSmartCurrentLimit(SwerveModuleConstants.kTurningMotorCurrentLimit);

		// Save the SPARK MAX configurations. If a SPARK MAX browns out during
		// operation, it will maintain the above configurations.
		m_turningSparkMax.burnFlash();

		m_chassisAngularOffset = chassisAngularOffset;
		m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());

		m_drivingTalonFX.setPosition(0);
	}

	/**
	 * Returns the current state of the module.
	 *
	 * @return The current state of the module.
	 */
	public SwerveModuleState getState() {
		// Apply chassis angular offset to the encoder position to get the position
		// relative to the chassis.
		return new SwerveModuleState(m_drivingTalonFX.getVelocity().getValue(),
			new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
	}

	/**
	 * Returns the current position of the module.
	 *
	 * @return The current position of the module.
	 */
	public SwerveModulePosition getPosition() {
		// Apply chassis angular offset to the encoder position to get the position
		// relative to the chassis.
		return new SwerveModulePosition(
			m_drivingTalonFX.getPosition().getValue(),
			new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
	}

	/**
	 * Sets the desired state for the module.
	 *
	 * @param desiredState Desired state with speed and angle.
	 */
	public void setDesiredState(SwerveModuleState desiredState) {
		// Apply chassis angular offset to the desired state.
		SwerveModuleState correctedDesiredState = new SwerveModuleState();
		correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
		correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

		// Optimize the reference state to avoid spinning further than 90 degrees.
		SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
			new Rotation2d(m_turningEncoder.getPosition()));

		// Command driving and turning SPARKS MAX towards their respective setpoints.
		m_drivingTalonFX.setControl(driveRequest.withVelocity(optimizedDesiredState.speedMetersPerSecond).withFeedForward(optimizedDesiredState.speedMetersPerSecond));
		m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

		m_desiredState = desiredState;
	}

	/** Zeroes all the SwerveModule encoders. */
	public void resetEncoders() {
		m_drivingTalonFX.setPosition(0);
	}

	public double steeringEncoder(){
		return m_turningEncoder.getPosition();
	}
}