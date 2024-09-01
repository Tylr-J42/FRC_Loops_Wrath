// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.OIConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shoulder;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;

public class RobotContainer {
    private Drivetrain drivetrain;
    private Shooter shooter;
    private Intake intake;
    private Shoulder shoulder;
    private Feeder feeder;

    private CommandXboxController driver;
    private CommandXboxController operator;

    public double tvecX;
    public double tvecY;
    public double tvecZ;

    public double targetTX;
    public double targetTY;

    private SendableChooser<Command> autoChooser;

    private static final String kAutoTabName = "Autonomous";

    public RobotContainer() {
        drivetrain = new Drivetrain(new Pose2d());
        shooter = new Shooter();
        intake = new Intake();
        feeder = new Feeder();
        shoulder = new Shoulder(() -> feeder.getBeamBreak());

        driver = new CommandXboxController(OIConstants.kDriverXboxUSB);
        operator = new CommandXboxController(OIConstants.kOperatorXboxUSB);

        autoChooser = AutoBuilder.buildAutoChooser();
        
        coprocessorNetworktables();
        shuffleboardSetup();
        configureBindings();
    }

    private void coprocessorNetworktables(){
        NetworkTableInstance rpi = NetworkTableInstance.getDefault();

        NetworkTable table = rpi.getTable("Fiducial");

        NetworkTableEntry tvecXEntry;
        NetworkTableEntry tvecYEntry;
        NetworkTableEntry tvecZEntry;

        Double camTiltDegrees = 30.0;
        
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if(DriverStation.Alliance.Red.equals(alliance)){
            tvecXEntry = table.getEntry("tag4tvecX");
            tvecYEntry = table.getEntry("tag4tvecY");
            tvecZEntry = table.getEntry("tag4tvecZ");
        }else{
            tvecXEntry = table.getEntry("tag7tvecX");
            tvecYEntry = table.getEntry("tag7tvecY");
            tvecZEntry = table.getEntry("tag7tvecZ");
        }

        tvecX = tvecXEntry.getDouble(0);
        tvecY = tvecYEntry.getDouble(0);
        tvecZ = tvecZEntry.getDouble(0);

        targetTX = Math.toDegrees(Math.atan(tvecZ/tvecX));
        targetTY = Math.toDegrees(Math.atan(tvecZ/tvecY))+camTiltDegrees;
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

        feeder.setDefaultCommand(feeder.manualFeedNote(() -> 0.0));
        shooter.setDefaultCommand(shooter.spinOff());
        shoulder.setDefaultCommand(shoulder.autoAimShoulder(targetTY));

        driver.leftTrigger().whileTrue(Commands.parallel(
            intake.runIntake(() -> 1.0),
            feeder.autoFeeding()
        ));

        driver.rightTrigger().whileTrue(
            Commands.parallel(feeder.manualFeedNote(() -> 1.0), 
            shooter.spinup()));

        operator.rightTrigger().toggleOnTrue(shooter.spinup());

        operator.a().whileTrue(Commands.parallel(shoulder.ampShotCommand(), shooter.spinup()));

        }

        private void shuffleboardSetup() {
            ShuffleboardTab autoTab = Shuffleboard.getTab(kAutoTabName);
            autoTab.add("Autonomous Selector", autoChooser)
                .withPosition(0, 0)
                .withSize(2, 1)
                .withWidget(BuiltInWidgets.kComboBoxChooser);

            Shuffleboard.selectTab(kAutoTabName);
        }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");    
    }
}
