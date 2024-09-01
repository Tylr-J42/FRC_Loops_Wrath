package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShoulderConstants;

public class Shoulder extends SubsystemBase{
    private CANSparkMax shoulder;

    private PIDController shoulderPID;
    private ArmFeedforward shoulderFF;

    private RelativeEncoder shoulderEncoder;
    
    private boolean beambreak;

    public Shoulder(BooleanSupplier beambreak){
        shoulder = new CANSparkMax(ShoulderConstants.kShoulderID, MotorType.kBrushless);

        shoulderEncoder = shoulder.getEncoder();

        shoulderPID = new PIDController(ShoulderConstants.kShoulderP, ShoulderConstants.kShoulderI, ShoulderConstants.kShoulderD);
        shoulderFF = new ArmFeedforward(ShoulderConstants.kFFShoulderS, ShoulderConstants.kFFShoulderG, ShoulderConstants.kFFShoulderV);
    }

    public void angleShoulder(DoubleSupplier angle){
        shoulder.setVoltage(shoulderPID.calculate(MathUtil.clamp(shoulderEncoder.getPosition(), 10.0, 210), angle.getAsDouble()) + shoulderFF.calculate(angle.getAsDouble(), 0));
    }

    public double getShoulderAngle(){
        return shoulderEncoder.getPosition();
    }

    public Command ampShotCommand(){
        return run(() -> angleShoulder(() -> ShoulderConstants.kAmpAngle));
    }

    public Command autoAimShoulder(DoubleSupplier ty){
        return run(() -> {
            if(!beambreak){
                angleShoulder(() -> ty.getAsDouble());
            }else{
                angleShoulder(() -> ShoulderConstants.kStowedAngle);
            }
        }
        );
    }

    public BooleanSupplier autoAimAtSetpoint(DoubleSupplier ty){
        if(Math.abs(getShoulderAngle()-ty.getAsDouble())<=3.0){
            return () -> true;
        }else{
            return () -> false;
        }
    }
}
