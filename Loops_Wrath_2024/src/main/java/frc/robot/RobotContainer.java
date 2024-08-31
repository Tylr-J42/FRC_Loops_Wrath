// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shoulder;

public class RobotContainer {
    private Drivetrain drivetrain;
    private Shooter shooter;
    private Intake intake;
    private Shoulder shoulder;
    private Feeder feeder;

    private CommandXboxController driver;
    private CommandXboxController operator;

    public RobotContainer() {
        drivetrain = new Drivetrain(new Pose2d());
        shooter = new Shooter();
        intake = new Intake();
        shoulder = new Shoulder();
        feeder = new Feeder();

        driver = new CommandXboxController(OIConstants.kDriverXboxUSB);
        operator = new CommandXboxController(OIConstants.kOperatorXboxUSB);

        configureBindings();
        coprocessorNetworktables();
    }

    private void coprocessorNetworktables(){
        NetworkTableInstance rpi = NetworkTableInstance.getDefault();

        NetworkTable table = rpi.getTable("Fiducial");
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

        driver.leftTrigger().whileTrue(intake.runIntake(() -> 1.0));
    
        driver.rightTrigger().whileTrue(feeder.feedNote(() -> 1.0));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");    
    }
}
