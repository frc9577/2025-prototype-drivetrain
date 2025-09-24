package frc.robot.factorys;

import java.util.Optional;

import com.studica.frc.AHRS;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import frc.robot.subsystems.VisionSubsystem;

public class VisionSubsystemFactory {
    public VisionSubsystemFactory() {
    }

    public Optional<VisionSubsystem> construct(DifferentialDrivePoseEstimator poseEstimator, AHRS gyro) {
        VisionSubsystem visionSubsystem = new VisionSubsystem(poseEstimator, gyro);
        return Optional.of(visionSubsystem);
    }
}
