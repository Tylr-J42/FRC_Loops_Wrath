package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase{
    CANSparkMax intakeRight;
    CANSparkMax intakeLeft;

    public Intake(){
        intakeRight = new CANSparkMax(IntakeConstants.kRightIntakeID, MotorType.kBrushless);
        intakeLeft = new CANSparkMax(IntakeConstants.kRightIntakeID, MotorType.kBrushless);
        
    }

    public void runIntake(double speed){
        intakeRight.set(speed);
        intakeLeft.set(speed);
    }
}
