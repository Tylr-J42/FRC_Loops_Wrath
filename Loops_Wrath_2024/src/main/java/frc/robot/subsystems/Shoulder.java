package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shoulder extends SubsystemBase{


    private PIDController shooterPivotPID;
    private ArmFeedforward shooterPivotFF;

    public Shoulder(){

    } 
}
