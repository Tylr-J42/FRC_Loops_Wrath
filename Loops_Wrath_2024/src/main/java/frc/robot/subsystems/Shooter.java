package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;


public class Shooter extends SubsystemBase{
    
    CANSparkMax leftShooter;
    CANSparkMax rightShooter;

    SparkPIDController rightShooterPID;
    SparkPIDController leftShooterPID;

    RelativeEncoder rightEncoder;
    RelativeEncoder leftEncoder;

    public Shooter(){
        leftShooter = new CANSparkMax(ShooterConstants.kLeftShooterID, MotorType.kBrushless);
        rightShooter = new CANSparkMax(ShooterConstants.kRightShooterID, MotorType.kBrushless);

        rightShooterPID = rightShooter.getPIDController();
        leftShooterPID = leftShooter.getPIDController();

        rightEncoder = rightShooter.getEncoder();
        leftEncoder = leftShooter.getEncoder();

        rightShooterPID.setFeedbackDevice(rightEncoder);
        leftShooterPID.setFeedbackDevice(leftEncoder);

        rightShooterPID.setP(ShooterConstants.kRightShooterP);
        rightShooterPID.setI(ShooterConstants.kRightShooterI);
        rightShooterPID.setD(ShooterConstants.kRightShooterD);

        leftShooterPID.setP(ShooterConstants.kLeftShooterP);
        leftShooterPID.setI(ShooterConstants.kLeftShooterI);
        leftShooterPID.setD(ShooterConstants.kLeftShooterD);

        rightShooterPID.setFF(ShooterConstants.kFFRight);
        leftShooterPID.setFF(ShooterConstants.kFFLeft);
    }

    public void setShooterSpeeds(double rightSpeed, double leftSpeed){
        rightShooterPID.setReference(rightSpeed, ControlType.kVelocity);
        leftShooterPID.setReference(leftSpeed, ControlType.kVelocity);
    }

    public double getShooterSpeedRight(){
        return rightEncoder.getVelocity();
    }

    public double getShooterSpeedLeft(){
        return leftEncoder.getVelocity();
    }
}
