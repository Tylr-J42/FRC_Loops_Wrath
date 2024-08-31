package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FeederConstants;

public class Feeder extends SubsystemBase{
    CANSparkMax feederMotor;

    public Feeder(){
        feederMotor = new CANSparkMax(FeederConstants.kFeederID, MotorType.kBrushless);
    }

    public void feedNote(double speed){
        feederMotor.set(speed);
    }
}
