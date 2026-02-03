package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public class VisionSubsystem extends SubsystemBase {

  private final VisionIO io;
  private final RobotState state;
  private boolean useVision = true;

  /** Creates a new vision subsystem. */
  public VisionSubsystem(VisionIO io, RobotState state) {
    this.io = io;
    this.state = state;
  }

  /**
   * @return new VisionPoseEstimateInField.
   */
  private VisionPoseEstimateInField cameraPoseEstimate(VisionPoseEstimateInField camera) {
    if (camera == null) {
      return null;
    }

    // Extraer datos existentes
    Pose2d pose2d = camera.pose();
    double timestampSeconds = camera.getTimestamp();
    Matrix<N3, N1> stdDevs = camera.getVisionMeasurementStdDevs();

    // 1. Validar StdDevs (equivalente a ambigüedad / calidad)
    if (stdDevs.get(0, 0) > VisionConstants.kLargeVariance
        || stdDevs.get(1, 0) > VisionConstants.kLargeVariance
        || stdDevs.get(2, 0) > VisionConstants.kLargeVariance) {
      return null;
    }

    // 2. Asegurar yaw razonable (equivalente a yaw threshold)
    if (Math.abs(pose2d.getRotation().getDegrees())
        > VisionConstants.kDefaultYawDiffThreshold * 10) {
      return null;
    }

    // 3. (Opcional) normalizar StdDevs si quieres controlarlos tú
    Matrix<N3, N1> normalizedStdDevs =
        VecBuilder.fill(
            Math.max(stdDevs.get(0, 0), VisionConstants.kDefaultNormThreshold),
            Math.max(stdDevs.get(1, 0), VisionConstants.kDefaultNormThreshold),
            Math.max(
                stdDevs.get(2, 0),
                Units.degreesToRadians(VisionConstants.kDefaultYawDiffThreshold)));

    // 4. Devolver estimación válida
    return new VisionPoseEstimateInField(pose2d, timestampSeconds, normalizedStdDevs, 0);
  }
}
