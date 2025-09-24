// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.studica.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
  private final DifferentialDrivePoseEstimator m_poseEstimator;
  private final AHRS m_gyro;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(DifferentialDrivePoseEstimator poseEstimator, AHRS gyro) {
    m_poseEstimator = poseEstimator;
    m_gyro = gyro;
  }

  // Returns the estimated position as a Pose2d
  public Pose2d getPose2d() {
    return m_poseEstimator.getEstimatedPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler 

    //// This is copy and pasted from limelight's documentation for testing. ////
    // First, tell Limelight your robot's current orientation
    double robotYaw = m_gyro.getYaw();  
    LimelightHelpers.SetRobotOrientation("", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Get the pose estimate
    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("");

    // Add it to your pose estimator
    m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
    m_poseEstimator.addVisionMeasurement(
        limelightMeasurement.pose,
        limelightMeasurement.timestampSeconds
    );
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
