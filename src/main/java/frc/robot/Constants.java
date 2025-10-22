// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class AutoConstants {
    // Module Config Stuff
    public static final double kWheelRadiusMeters = 0.0;
    public static final double kMaxDriveVelocityMPS = 0.0;
    public static final double kWheelCOF = 0.0;
    public static final int kNumMotors = 4;
    public static final double kDriveCurrentLimit = 0.0;

    public static final DCMotor kDriveMotor = DCMotor.getKrakenX60(kNumMotors);
    public static final ModuleConfig kMoudleConfig = new ModuleConfig(kWheelRadiusMeters, kMaxDriveVelocityMPS, kWheelCOF, kDriveMotor, kDriveCurrentLimit, kNumMotors);

    // Robot Config Stuff
    public static final double kMassKG = 0.0;
    public static final double kMOI = 0.0; // Moment of Intertia
    public static final double kTrackWithMeters = 0.0;

    public static final RobotConfig kRobotConfig = new RobotConfig(kMassKG, kMOI, kMoudleConfig, kTrackWithMeters);
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DrivetrainConstants {
    public static final int kLeftMotorCANID = 10;
    public static final int kOptionalLeftMotorCANID = 11;

    public static final int kRightMotorCANID = 20;
    public static final int kOptionalRightMotorCANID = 21;

    public static final double kTurnDivider = 2;
    public static final double kSpeedDivider = 2.5;

    // Auto PID stuff
    public static final double kV = 0; // Add x V output to overcome static friction
    public static final double kS = 0; // A velocity target of 1 rps results in xV output
    public static final double kP = 0.3; // An error of 1 rotation results in x V output
    public static final double kI = 0.0;
    public static final double kD = 0.1; // A velocity of 1 rps results in x V output
    public static final double PeakVoltage = 10.0;

    public static final int maxVelocity = 30; // rps/s
    public static final int maxAcceleration = 50; // rps

    // For Auto Potentially
    public static boolean kLeftPositiveMovesForward = true;
    public static boolean kRightPositiveMovesForward = true;

    // The distance travelled for a single rotation of the Kraken output shaft.
    public static final double kDrivetrainGearRatio = (8.46/0.478536);
    public static final double kWheelRadiusMeters = (4.0 / 2.0) * 0.0254; // Four Inch Wheels
    public static final double kWheelCircumfrance = 2 * Math.PI * DrivetrainConstants.kWheelRadiusMeters;

    // SmartDashboard update frequency for drive subsystem state in 20ms counts.
    public static final int kTicksPerUpdate = 5;

    // The track width in meters.
    public static final double trackWidthMeters = 1; // TODO: Set Value!
  }
}
