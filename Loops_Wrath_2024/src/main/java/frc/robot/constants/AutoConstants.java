package frc.robot.constants;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4; // max capable = 4.6
    public static final double kMaxAccelerationMetersPerSecondSquared = 4; // max capable = 7.4
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 5.5;//5;
    public static final double kPYController = 5.5;//5;
    public static final double kPThetaController = 5;//0.5; // needs to be separate from heading control

    public static final double kPHeadingController = 0.02; // for heading control NOT PATHING
    public static final double kDHeadingController = 0.0025;

    public static final HolonomicPathFollowerConfig kPFConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(kPXController), 
        new PIDConstants(kPThetaController), 
        kMaxSpeedMetersPerSecond, 
        Units.inchesToMeters(Math.sqrt(Math.pow(14-1.75, 2)+ Math.pow(14-1.75, 2))), 
        new ReplanningConfig()
    );

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
}