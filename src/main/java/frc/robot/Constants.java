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
    // TODO: Run SYS ID and fill in!
    public static final double kWheelRadiusMeters = 0.0;
    public static final double kMaxDriveVelocityMPS = 0.0;
    public static final double kWheelCOF = 0.0;
    public static final int kNumMotors = 4;
    public static final double kDriveCurrentLimit = 0.0;

    public static final DCMotor kDriveMotor = DCMotor.getKrakenX60(kNumMotors);
    public static final ModuleConfig kMoudleConfig = new ModuleConfig(kWheelRadiusMeters, kMaxDriveVelocityMPS, kWheelCOF, kDriveMotor, kDriveCurrentLimit, kNumMotors);

    // Robot Config Stuff
    // TODO: Run SYS ID and fill in!
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
    public static final int kRightMotorCANID = 20;
  }
}
