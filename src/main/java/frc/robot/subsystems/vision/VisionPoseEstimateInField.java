package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionPoseEstimateInField {
  private final Pose2d robotPose;
  private final double timestamp;
  private final Matrix<N3, N1> visionMeasurementStdDevs;
  private final int numTags;

  // implementar este archivo para la odometría

  public VisionPoseEstimateInField(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs,
      int numTags) {
    this.robotPose = visionRobotPoseMeters;
    this.timestamp = timestampSeconds;
    this.visionMeasurementStdDevs = visionMeasurementStdDevs;
    this.numTags = numTags;
  }

  public Pose2d getRobotPose() {
    return robotPose;
  }

  public double getTimestamp() {
    return timestamp;
  }

  public Matrix<N3, N1> getVisionMeasurementStdDevs() {
    return visionMeasurementStdDevs;
  }

  public int getNumTags() {
    return numTags;
  }
}
