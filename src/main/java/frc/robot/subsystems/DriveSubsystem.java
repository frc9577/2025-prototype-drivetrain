// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.*;
import frc.robot.commands.ArcadeDriveCommand;

public class DriveSubsystem extends SubsystemBase {
  private final TalonFX m_rightMotor = new TalonFX(DrivetrainConstants.kRightMotorCANID);
  private TalonFX m_optionalRightMotor;

  private final TalonFX m_leftMotor  = new TalonFX(DrivetrainConstants.kLeftMotorCANID);
  private TalonFX m_optionalLeftMotor; 

  private DifferentialDrive m_Drivetrain;

  private double m_leftSpeed  = 0.0;
  private double m_rightSpeed = 0.0;

  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);
  private final DifferentialDrivePoseEstimator m_poseEstimator;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(DifferentialDrivePoseEstimator poseEstimator) { 
    m_poseEstimator = poseEstimator;

    // Right Motor
    TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();
    rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    rightMotorConfig.Voltage.withPeakForwardVoltage(Volts.of(DrivetrainConstants.PeakVoltage))
    .withPeakReverseVoltage(Volts.of(-DrivetrainConstants.PeakVoltage));

    m_rightMotor.getConfigurator().apply(rightMotorConfig);    
    SendableRegistry.setName(m_rightMotor, "DriveSubsystem", "rightMotor");

    // Optional Right Motor
    try {
      m_optionalRightMotor = new TalonFX(DrivetrainConstants.kOptionalRightMotorCANID);
  
      // Setting up Config
      TalonFXConfiguration optionalRightMotorConfig = new TalonFXConfiguration();
      optionalRightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      optionalRightMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      optionalRightMotorConfig.Voltage.withPeakForwardVoltage(Volts.of(DrivetrainConstants.PeakVoltage))
      .withPeakReverseVoltage(Volts.of(-DrivetrainConstants.PeakVoltage));

      // Saving
      m_optionalRightMotor.getConfigurator().apply(optionalRightMotorConfig);

      // Setting as a follwer
      m_optionalRightMotor.setControl(
        new Follower(m_rightMotor.getDeviceID(), false)
      );
    }
    catch (Exception e) {
      e.printStackTrace();
    }

    // Left Motor
    TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
    leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    leftMotorConfig.Voltage.withPeakForwardVoltage(Volts.of(DrivetrainConstants.PeakVoltage))
    .withPeakReverseVoltage(Volts.of(-DrivetrainConstants.PeakVoltage));

    m_leftMotor.getConfigurator().apply(leftMotorConfig);
    SendableRegistry.setName(m_leftMotor, "DriveSubsystem", "leftMotor");

    // Optional Left Motor
    try {
      m_optionalLeftMotor = new TalonFX(DrivetrainConstants.kOptionalLeftMotorCANID);

      // Setting up Config
      TalonFXConfiguration optionalLeftMotorConfig = new TalonFXConfiguration();
      optionalLeftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      optionalLeftMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      optionalLeftMotorConfig.Voltage.withPeakForwardVoltage(Volts.of(DrivetrainConstants.PeakVoltage))
      .withPeakReverseVoltage(Volts.of(-DrivetrainConstants.PeakVoltage));

      // Saving
      m_optionalLeftMotor.getConfigurator().apply(optionalLeftMotorConfig);

      // Setting as follower
      m_optionalLeftMotor.setControl(
        new Follower(m_leftMotor.getDeviceID(), false)
      );
    }
    catch (Exception e)
    {
      // TODO: There are really two cases you want to catch. The first, when the follower
      // motor controller doesn't exist, isn't an error. The second, where the motor exists
      // but one of the later configuration calls fails, is an error. Generally, you would
      // only dump a stack trace in error cases and you definitely don't want to do this in
      // normal operation. I would suggest splitting this block into two try/except chunks,
      // on that catches the missing controller and just outputs a status message to the log
      // indicating that only one motor is in use, and the other catching the real errors
      // and dumping the stack trace.
      e.printStackTrace();
    }

    // Zeroing the encoders
    m_leftMotor.setPosition(0);
    m_rightMotor.setPosition(0);

    // Setting up the drive train
    m_Drivetrain = new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);
    SendableRegistry.setName(m_Drivetrain, "DriveSubsystem", "Drivetrain");   
  }

  public void initDefaultCommand(CommandXboxController Controller)
  {
    setDefaultCommand(new ArcadeDriveCommand(this, Controller));
  }

  public void setArcadeSpeeds(double joystickInput, double rotationInput)
  {
    m_leftSpeed = (joystickInput / DrivetrainConstants.kSpeedDivider);
    m_rightSpeed = (rotationInput / DrivetrainConstants.kTurnDivider);

    // NOTE: We are making our own custom input modifications
    //m_leftSpeed = Math.pow(m_leftSpeed, 3);
    //m_rightSpeed = Math.pow(m_rightSpeed, 3);

    m_Drivetrain.arcadeDrive(m_leftSpeed, m_rightSpeed, true);
  }

  public double getSpeed(boolean bLeft)
  {
    if (bLeft)
    {
      return m_leftSpeed;
    }
    else
    {
      return m_rightSpeed;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
