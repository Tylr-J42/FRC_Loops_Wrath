package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

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

    public Shoulder(BooleanSupplier beambreak){
        shoulder = new CANSparkMax(ShoulderConstants.kShoulderID, MotorType.kBrushless);

        shoulderEncoder = shoulder.getEncoder();

        shoulderPID = new PIDController(ShoulderConstants.kShoulderP, ShoulderConstants.kShoulderI, ShoulderConstants.kShoulderD);
        shoulderFF = new ArmFeedforward(ShoulderConstants.kFFShoulderS, ShoulderConstants.kFFShoulderG, ShoulderConstants.kFFShoulderV);
    }

    public void angleShoulder(DoubleSupplier angle){
        shoulder.setVoltage(shoulderPID.calculate(shoulderEncoder.getPosition(), angle.getAsDouble()) + shoulderFF.calculate(angle.getAsDouble(), 0));
    }

    public double getShoulderAngle(){
        return shoulderEncoder.getPosition();
    }

    public Command ampShotCommand(){
        return run(() -> angleShoulder(() -> ShoulderConstants.kAmpAngle));
    }

    public Command autoAimShoulder(){
        return run(


        );
    }
}
