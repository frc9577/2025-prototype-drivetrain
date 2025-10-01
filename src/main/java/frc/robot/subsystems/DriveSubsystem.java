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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.*;
import frc.robot.commands.ArcadeDriveCommand;

public class DriveSubsystem extends SubsystemBase {
  private TalonFX m_rightMotor;
  private TalonFX m_optionalRightMotor;

  private TalonFX m_leftMotor;
  private TalonFX m_optionalLeftMotor; 

  private DifferentialDrive m_Drivetrain;

  private double m_leftSpeed  = 0.0;
  private double m_rightSpeed = 0.0;

  private final DifferentialDrivePoseEstimator m_poseEstimator;
  private final AHRS m_gyro;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(DifferentialDrivePoseEstimator poseEstimator, AHRS gyro, TalonFX rightMotor, TalonFX leftMotor) { 
    m_poseEstimator = poseEstimator;
    m_gyro = gyro;
    m_rightMotor = rightMotor;
    m_leftMotor = leftMotor;

    // Right Motor
    TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();
    rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    rightMotorConfig.Voltage.withPeakForwardVoltage(Volts.of(DrivetrainConstants.PeakVoltage))
    .withPeakReverseVoltage(Volts.of(-DrivetrainConstants.PeakVoltage));

    m_rightMotor.getConfigurator().apply(rightMotorConfig);    
    SendableRegistry.setName(m_rightMotor, "DriveSubsystem", "rightMotor");

    // Left Motor
    TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
    leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    leftMotorConfig.Voltage.withPeakForwardVoltage(Volts.of(DrivetrainConstants.PeakVoltage))
    .withPeakReverseVoltage(Volts.of(-DrivetrainConstants.PeakVoltage));

    m_leftMotor.getConfigurator().apply(leftMotorConfig);
    SendableRegistry.setName(m_leftMotor, "DriveSubsystem", "leftMotor");

    // Zeroing the encoders
    m_leftMotor.setPosition(0);
    m_rightMotor.setPosition(0);

    // Setting up the drive train
    m_Drivetrain = new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);
    SendableRegistry.setName(m_Drivetrain, "DriveSubsystem", "Drivetrain");   
  }

  public void setFollowers(TalonFX optionalRight, TalonFX optionalLeft) {
    m_optionalRightMotor = optionalRight;
  
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

    m_optionalLeftMotor = optionalLeft;

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

  // Wrapping robot position inside of getposition
  public Pose2d getPose2d(){
    return m_poseEstimator.getEstimatedPosition();
  }

  // Resets the drivetrains pose estimator to zero.
  public void resetPose(){
    m_leftMotor.setPosition(0);
    m_rightMotor.setPosition(0);
    m_poseEstimator.resetPose(Pose2d.kZero);
  }

  // Returns a robot relative ChassisSpeeds object based on the avrg linear velocity
  // in meters per second and avrg anglear velocity in readians per second
  public ChassisSpeeds getRobotRelativeSpeeds(){
    // Linear Velocity in meters per second
    double leftMPS = m_leftMotor.getVelocity().getValueAsDouble() * DrivetrainConstants.kWheelCircumfrance;
    double rightMPS = m_rightMotor.getVelocity().getValueAsDouble() * DrivetrainConstants.kWheelCircumfrance;

    // Anglear Velocity in radians per second
    double leftRPS = leftMPS/DrivetrainConstants.kWheelDistanceFromCenterOfRotation;
    double rightRPS = rightMPS/DrivetrainConstants.kWheelDistanceFromCenterOfRotation;

    // Avrging them togther
    double linearVelocity = (leftMPS+rightMPS)/2;
    double AnglearVelocity = (leftRPS+rightRPS)/2;

    // Making Chassis Speeds
    return new ChassisSpeeds(linearVelocity, 0, AnglearVelocity);
  }

  // Gives the drivetrain a new drive command based on a robot relative
  // chassis speed object.
  public void driveRobotRelative(ChassisSpeeds relativeChassisSpeed){
    m_Drivetrain.arcadeDrive(relativeChassisSpeed.vxMetersPerSecond, 
                            relativeChassisSpeed.omegaRadiansPerSecond);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    m_poseEstimator.update(
        m_gyro.getRotation2d(), 
        m_leftMotor.getPosition().getValueAsDouble(), 
        m_rightMotor.getPosition().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
