// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.studica.frc.AHRS;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.utils.PIDControl;

public class DriveSubsystem extends SubsystemBase {
  private TalonFX m_rightMotor;
  private TalonFX m_optionalRightMotor;

  private TalonFX m_leftMotor;
  private TalonFX m_optionalLeftMotor; 

  private DifferentialDrive m_Drivetrain;

  private double m_leftSpeed  = 0.0;
  private double m_rightSpeed = 0.0;

  private final DifferentialDriveKinematics m_kinematics;
  private final DifferentialDrivePoseEstimator m_poseEstimator;
  private final AHRS m_gyro;

  PIDControl m_leftPidControl = new PIDControl(
    m_leftMotor, 
    DrivetrainConstants.kLeftPositiveMovesForward
  );

  PIDControl m_rightPidControl = new PIDControl(
    m_rightMotor, 
    DrivetrainConstants.kRightPositiveMovesForward
  );

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(DifferentialDrivePoseEstimator poseEstimator, DifferentialDriveKinematics kinematics, AHRS gyro, TalonFX rightMotor, TalonFX leftMotor) { 
    m_poseEstimator = poseEstimator;
    m_kinematics = kinematics;
    m_gyro = gyro;
    m_rightMotor = rightMotor;
    m_leftMotor = leftMotor;

    // Right Motor
    TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();
    rightMotorConfig.Slot0.kV = DrivetrainConstants.kV;
    rightMotorConfig.Slot0.kS = DrivetrainConstants.kS;
    rightMotorConfig.Slot0.kP = DrivetrainConstants.kP;
    rightMotorConfig.Slot0.kI = DrivetrainConstants.kI; // No output for integrated error
    rightMotorConfig.Slot0.kD = DrivetrainConstants.kD; // A velocity of 1 rps results in 0.1 V output
    rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    rightMotorConfig.Voltage.withPeakForwardVoltage(Volts.of(DrivetrainConstants.PeakVoltage))
    .withPeakReverseVoltage(Volts.of(-DrivetrainConstants.PeakVoltage));

    m_rightMotor.getConfigurator().apply(rightMotorConfig);    
    SendableRegistry.setName(m_rightMotor, "DriveSubsystem", "rightMotor");

    // Left Motor
    TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
    leftMotorConfig.Slot0.kV = DrivetrainConstants.kV;
    leftMotorConfig.Slot0.kS = DrivetrainConstants.kS;
    leftMotorConfig.Slot0.kP = DrivetrainConstants.kP;
    leftMotorConfig.Slot0.kI = DrivetrainConstants.kI; // No output for integrated error
    leftMotorConfig.Slot0.kD = DrivetrainConstants.kD; // A velocity of 1 rps results in 0.1 V output
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
    
      // Gyro setup
      m_gyro.zeroYaw();
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
  // Currently we are asuming that their is no scale for the motors, we cannot find anywhere to set the scale.
  public ChassisSpeeds getRobotRelativeSpeeds(){
    // Linear Velocity in meters per second
    double leftMPS = m_leftMotor.getVelocity().getValueAsDouble() * DrivetrainConstants.kDrivetrainGearRatio * DrivetrainConstants.kWheelCircumfrance;
    double rightMPS = m_rightMotor.getVelocity().getValueAsDouble() * DrivetrainConstants.kDrivetrainGearRatio * DrivetrainConstants.kWheelCircumfrance;

    // Make wheelSpeeds object from MPS & converts it to chasis speeds
    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(leftMPS, rightMPS);
    return m_kinematics.toChassisSpeeds(wheelSpeeds);
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
