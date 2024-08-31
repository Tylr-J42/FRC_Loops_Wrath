package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.utilities.MAXSwerveModule;
import frc.robot.utilities.SwerveUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    // Create MAXSwerveModules
    private final MAXSwerveModule m_frontLeft;
    private final MAXSwerveModule m_frontRight;
    private final MAXSwerveModule m_rearLeft;
    private final MAXSwerveModule m_rearRight; 

    // The gyro sensor
    private final AHRS m_gyro;
    //private final ADXRS450_Gyro m_gyro;

    private final SlewRateLimiter m_magLimiter;
    private final SlewRateLimiter m_rotLimiter;

    // Odometry class for tracking robot pose
    private final SwerveDrivePoseEstimator m_poseEstimator;

    private boolean fieldRelativeControl;

    // Slew rate filter variables for controlling lateral acceleration
    private double m_currentRotation;
    private double m_currentTranslationDir;
    private double m_currentTranslationMag;

    private double m_prevTime;

    private double gyroOffset;

    /** Creates a new DriveSubsystem. */
    public Drivetrain(Pose2d initialPose) {
        m_frontLeft = new MAXSwerveModule(
            DrivetrainConstants.kFrontLeftDrivingCanId,
            DrivetrainConstants.kFrontLeftTurningCanId,
            DrivetrainConstants.kFrontLeftChassisAngularOffset
        );
        m_frontRight = new MAXSwerveModule(
            DrivetrainConstants.kFrontRightDrivingCanId,
            DrivetrainConstants.kFrontRightTurningCanId,
            DrivetrainConstants.kFrontRightChassisAngularOffset
        );

        m_rearLeft = new MAXSwerveModule(
            DrivetrainConstants.kRearLeftDrivingCanId,
            DrivetrainConstants.kRearLeftTurningCanId,
            DrivetrainConstants.kBackLeftChassisAngularOffset
        );

        m_rearRight = new MAXSwerveModule(
            DrivetrainConstants.kRearRightDrivingCanId,
            DrivetrainConstants.kRearRightTurningCanId,
            DrivetrainConstants.kBackRightChassisAngularOffset
        );

        // TODO The NavX is not centered in the robot, it's output probably needs to be translated
        m_gyro = new AHRS(SPI.Port.kMXP);
       // m_gyro = new ADXRS450_Gyro();
        //m_gyro.calibrate();

        m_magLimiter = new SlewRateLimiter(DrivetrainConstants.kMagnitudeSlewRate);
        m_rotLimiter = new SlewRateLimiter(DrivetrainConstants.kRotationalSlewRate);

        m_poseEstimator = new SwerveDrivePoseEstimator(
            DrivetrainConstants.kDriveKinematics,
            Rotation2d.fromDegrees(getGyroAngle()),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
            },
            initialPose
        );

        fieldRelativeControl = true;

        m_currentRotation = 0.0;
        m_currentTranslationDir = 0.0;
        m_currentTranslationMag = 0.0;
        m_prevTime =  WPIUtilJNI.now() * 1e-6;

        AutoBuilder.configureHolonomic(
            this::getPose, 
            this::resetOdometry, 
            this::getCurrentChassisSpeeds, 
            this::driveWithChassisSpeeds, 
            AutoConstants.kPFConfig, 
            () -> {
                Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
            }, 
            this
        );

        gyroOffset = DrivetrainConstants.kRobotStartOffset;

    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_poseEstimator.update(
            Rotation2d.fromDegrees(getGyroAngle()),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
            }
        );

    }

    public double getRobotStartOffset() {
        return gyroOffset;
    }

    public void setRobotStartOffset(double offset) {
        gyroOffset = offset;
    }

    public boolean isFieldRelativeControl() {
        return fieldRelativeControl;
    }

    public Command toggleFieldRelativeControl() {
        return runOnce(() -> {
            fieldRelativeControl = !fieldRelativeControl;
        });
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public double getPoseX(){
        return m_poseEstimator.getEstimatedPosition().getX();
    }

    public double getPoseY(){
        return m_poseEstimator.getEstimatedPosition().getY();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_gyro.zeroYaw();
        m_gyro.setAngleAdjustment(pose.getRotation().getDegrees());

        m_poseEstimator.resetPosition(
            Rotation2d.fromDegrees(getGyroAngle()),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
            },
            pose
        );
    }

    public ChassisSpeeds getCurrentChassisSpeeds() {
        return DrivetrainConstants.kDriveKinematics.toChassisSpeeds(
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_rearLeft.getState(),
            m_rearRight.getState()
        );
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     * @param rateLimit     Whether to enable rate limiting for smoother control.
     */
    public void drive(double xSpeed, double ySpeed, double rot, BooleanSupplier fieldRelative, boolean rateLimit) {
        double xSpeedCommanded;
        double ySpeedCommanded;

        if (rateLimit) {
            // Convert XY to polar for rate limiting
            double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
            double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

            // Calculate the direction slew rate based on an estimate of the lateral acceleration
            double directionSlewRate;
            if (m_currentTranslationMag != 0.0) {
                directionSlewRate = Math.abs(DrivetrainConstants.kDirectionSlewRate / m_currentTranslationMag);
            } else {
                directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
            }
            
            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - m_prevTime;
            double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);

            if (angleDif < 0.45*Math.PI) {
                m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
                m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
            } else if (angleDif > 0.85*Math.PI) {
                if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
                    // keep currentTranslationDir unchanged
                    m_currentTranslationMag = m_magLimiter.calculate(0.0);
                } else {
                    m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
                    m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
                }
            } else {
                m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
                m_currentTranslationMag = m_magLimiter.calculate(0.0);
            }

            m_prevTime = currentTime;
            
            xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
            ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
            m_currentRotation = m_rotLimiter.calculate(rot);
        } else {
            xSpeedCommanded = xSpeed;
            ySpeedCommanded = ySpeed;
            m_currentRotation = rot;
        }

        if(fieldRelative.getAsBoolean() &&
        DriverStation.getAlliance().isPresent() &&
        DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
            xSpeed = -xSpeed;
            ySpeed = -ySpeed;
        }

        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeedCommanded * DrivetrainConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeedCommanded * DrivetrainConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = m_currentRotation * DrivetrainConstants.kMaxAngularSpeed;

        var swerveModuleStates = DrivetrainConstants.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
            fieldRelative.getAsBoolean()
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, getHeading())
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered),
                0.2)
                );
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, DrivetrainConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    public void driveWithChassisSpeeds(ChassisSpeeds speeds) {
        //speeds.omegaRadiansPerSecond = -speeds.omegaRadiansPerSecond;
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.2);
        SwerveModuleState[] newStates = DrivetrainConstants.kDriveKinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(newStates, DrivetrainConstants.kMaxSpeedMetersPerSecond);
        setModuleStates(newStates);
        
        /*
        //speeds.omegaRadiansPerSecond = -speeds.omegaRadiansPerSecond;
        SwerveModuleState[] states = DrivetrainConstants.kDriveKinematics.toSwerveModuleStates(speeds);

        m_frontLeft.setDesiredState(states[0]);
        m_frontRight.setDesiredState(states[1]);
        m_rearLeft.setDesiredState(states[2]);
        m_rearRight.setDesiredState(states[3]);
        */
    }

    public Command teleopCommand(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotation, double deadband) {
        return run(() -> {
            drive(
                MathUtil.applyDeadband(xSpeed.getAsDouble(), deadband), 
                MathUtil.applyDeadband(ySpeed.getAsDouble(), deadband), 
                MathUtil.applyDeadband(rotation.getAsDouble(), deadband), 
                () -> fieldRelativeControl, 
                true
            );
        });
    }

    public Command driveCardinal(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier pov, double deadband) {
        PIDController controller = new PIDController(
            AutoConstants.kPHeadingController, 
            0, 
            AutoConstants.kDHeadingController
        );
        controller.enableContinuousInput(-180, 180);
        
        return new PIDCommand(
            controller, 
            this::getGyroAngle, 
            pov::getAsDouble, 
            (output) -> {
                this.drive(
                    -MathUtil.applyDeadband(xSpeed.getAsDouble(), deadband), 
                    -MathUtil.applyDeadband(ySpeed.getAsDouble(), deadband), 
                    output, 
                    () -> fieldRelativeControl, 
                    true
                );
            }, 
            this);
    }

    public Command autoAim(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier error, double deadband) {
        PIDController controller = new PIDController(
            AutoConstants.kPHeadingController, 
            0, 
            AutoConstants.kDHeadingController
        );
        controller.enableContinuousInput(-180, 180);

        double setpoint = this.getGyroAngle()+error.getAsDouble();
        
        return new PIDCommand(
            controller, 
            this::getGyroAngle, 
            setpoint, 
            (output) -> {
                this.drive(
                    -MathUtil.applyDeadband(xSpeed.getAsDouble(), deadband), 
                    -MathUtil.applyDeadband(ySpeed.getAsDouble(), deadband), 
                    output, 
                    () -> fieldRelativeControl, 
                    true
                );
            }, 
            this);
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public Command setXCommand() {
        return runOnce(this::setX);
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates, DrivetrainConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        Pose2d tempPose = m_poseEstimator.getEstimatedPosition();

        Rotation2d tempRotation = Rotation2d.fromDegrees(0);

        resetOdometry(
            new Pose2d(
                tempPose.getX(), 
                tempPose.getY(), 
                tempRotation
            )
        );
    }

    public Command zeroHeadingCommand() {
        return runOnce(this::zeroHeading);
    }

    public void offsetZero(Pose2d angle){
        resetOdometry(angle);
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(getGyroAngle());
    }

    public double getGyroAngle(){
        return m_gyro.getAngle() * (DrivetrainConstants.kGyroReversed ? -1.0 : 1.0);
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return m_gyro.getRate() * (DrivetrainConstants.kGyroReversed ? -1.0 : 1.0);
    }

    public double getEncoderSteering(){
        return m_frontLeft.steeringEncoder();
    }
}
