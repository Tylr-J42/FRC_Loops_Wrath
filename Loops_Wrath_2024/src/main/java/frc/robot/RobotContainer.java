// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shoulder;

public class RobotContainer {
    private Drivetrain drivetrain;
    private Shooter shooter;
    private Intake intake;
    private Shoulder shoulder;

    private XboxController driver;
    private XboxController operator;

    public RobotContainer() {
        drivetrain = new Drivetrain(new Pose2d());
        shooter = new Shooter();
        intake = new Intake();
        shoulder = new Shoulder();

        driver = new XboxController(OIConstants.kDriverXboxUSB);
        operator = new XboxController(OIConstants.kOperatorXboxUSB)

        configureBindings();
    }

    private void configureBindings() {
        
        drivetrain.setDefaultCommand(
            drivetrain.teleopCommand(
                () -> { return driver.getLeftY();}, 
                () -> { return driver.getLeftX();}, 
                () -> { return driver.getRightX();}, 
                OIConstants.kTeleopDriveDeadband
            )
        );

    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
