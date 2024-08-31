package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase{
    CANSparkMax intakeRight;
    CANSparkMax intakeLeft;

    public Intake(){
        intakeRight = new CANSparkMax(IntakeConstants.kRightIntakeID, MotorType.kBrushless);
        intakeLeft = new CANSparkMax(IntakeConstants.kRightIntakeID, MotorType.kBrushless);
        
    }

    public Command runIntake(DoubleSupplier speed){
        return run(() -> {
        intakeRight.set(speed.getAsDouble());
        intakeLeft.set(speed.getAsDouble());
        }
        );
    }
}
