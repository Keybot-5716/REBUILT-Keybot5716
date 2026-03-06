package frc.robot.subsystems.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {
  private final VisionIO io;
  private final RobotState state;
  private final VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();
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
    Pose2d pose2d = camera.getRobotPose();
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

  @Override
  public void periodic() {
    double startTime = Timer.getFPGATimestamp();
    io.readInputs(inputs);

    logCameraInputs("Vision/CameraA", inputs.cameraA);

    var maybeMTA = processCamera(inputs.cameraA, "CameraA", VisionConstants.kRobotToCameraA);

    if (!useVision) {
      Logger.recordOutput("Vision/usingVision", false);
      Logger.recordOutput("Vision/exclusiveTagId", state.getExclusiveTag().orElse(-1));
      Logger.recordOutput("Vision/latencyPeriodicSec", Timer.getFPGATimestamp() - startTime);
      return;
    }

    Logger.recordOutput("Vision/usingVision", true);

    /*
    Optional<VisionPoseEstimateInField> accepted = Optional.empty();
    accepted = Optional.of(cameraPoseEstimate(maybeMTA.get()));
    if (maybeMTA.isPresent()) {
      accepted = maybeMTA.map(this::cameraPoseEstimate).flatMap(Optional::ofNullable);
    }*/
    Optional<VisionPoseEstimateInField> accepted =
        maybeMTA.map(this::cameraPoseEstimate).flatMap(Optional::ofNullable);
    accepted.ifPresent(
        est -> {
          Logger.recordOutput("Vision/fusedAccepted", est.getRobotPose());
          state.updateMegatagEstimate(est);
        });

    Logger.recordOutput("Vision/exclusiveTagId", state.getExclusiveTag().orElse(-1));
    Logger.recordOutput("Vision/latencyPeriodicSec", Timer.getFPGATimestamp() - startTime);

    Logger.recordOutput("Vision/HasMaybeMTA", maybeMTA.isPresent());
    Logger.recordOutput("Vision/HasAccepted", accepted.isPresent());
  }

  private void logCameraInputs(String prefix, VisionIO.VisionIOInputs.CameraInputs cam) {
    Logger.recordOutput(prefix + "/SeesTarget", cam.seesTarget);
    Logger.recordOutput(prefix + "/MegatagCount", cam.megatagcount);

    if (DriverStation.isDisabled()) {
      SmartDashboard.putBoolean(prefix + "/SeesTarget", cam.seesTarget);
      SmartDashboard.putNumber(prefix + "/MegatagCount", cam.megatagcount);
    }

    if (cam.pose3d != null) {
      Logger.recordOutput(prefix + "/Pose3d", cam.pose3d);
    }

    if (cam.megatagPoseEstimate != null) {
      Logger.recordOutput(prefix + "/MegatagPoseEstimate", cam.megatagPoseEstimate.fieldToRobot());
      Logger.recordOutput(prefix + "/Quality", cam.megatagPoseEstimate.quality());
      Logger.recordOutput(prefix + "/AvgTagArea", cam.megatagPoseEstimate.avgTagArea());
    }

    if (cam.fiducialAprilTagObservation != null) {
      Logger.recordOutput(prefix + "/FiducialCount", cam.fiducialAprilTagObservation.length);
    }
  }

  private Optional<VisionPoseEstimateInField> processCamera(
      VisionIO.VisionIOInputs.CameraInputs cam, String label, Transform2d robotToCamera) {

    String logPrefix = "Vision/" + label;

    if (!cam.seesTarget) {
      return Optional.empty();
    }

    Optional<VisionPoseEstimateInField> estimate = Optional.empty();

    if (cam.megatagPoseEstimate != null) {
      Optional<VisionPoseEstimateInField> mtEstimate =
          processMegatagPoseEstimate(cam.megatagPoseEstimate, cam, logPrefix);

      mtEstimate.ifPresent(
          est -> Logger.recordOutput(logPrefix + "/AcceptedMegatagEstimate", est.getRobotPose()));

      Optional<VisionPoseEstimateInField> gyroEstimate =
          fuseWithGyro(cam.megatagPoseEstimate, cam, logPrefix);

      gyroEstimate.ifPresent(
          est -> Logger.recordOutput(logPrefix + "/FuseWithGyroEstimate", est.getRobotPose()));

      // Prefer Megatag when available
      if (mtEstimate.isPresent()) {
        estimate = mtEstimate;
        Logger.recordOutput(logPrefix + "/AcceptMegatag", true);
        Logger.recordOutput(logPrefix + "/AcceptGyro", false);
      } else if (gyroEstimate.isPresent()) {
        estimate = gyroEstimate;
        Logger.recordOutput(logPrefix + "/AcceptMegatag", false);
        Logger.recordOutput(logPrefix + "/AcceptGyro", true);
      } else {
        Logger.recordOutput(logPrefix + "/AcceptMegatag", false);
        Logger.recordOutput(logPrefix + "/AcceptGyro", false);
      }
    }

    return estimate;
  }

  private Optional<VisionPoseEstimateInField> fuseWithGyro(
      MegaTagPoseEstimate poseEstimate,
      VisionIO.VisionIOInputs.CameraInputs cam,
      String logPrefix) {

    if (poseEstimate.timestampSeconds() <= state.lastMegatagTimestamp()) {
      return Optional.empty();
    }

    // Use Megatag directly when 2 or more tags are visible
    if (poseEstimate.fiducialIds().length > 1) {
      return Optional.empty();
    }

    // Reject if the robot is yawing rapidly (time‑sync unreliable)
    final double kHighYawLookbackS = 0.3;
    final double kHighYawVelocityRadS = 5.0;

    if (state
            .getMaxAbsDriveYawAngularVelocityInRange(
                poseEstimate.timestampSeconds() - kHighYawLookbackS,
                poseEstimate.timestampSeconds())
            .orElse(Double.POSITIVE_INFINITY)
        > kHighYawVelocityRadS) {
      return Optional.empty();
    }

    var priorPose = state.getFieldToRobot(poseEstimate.timestampSeconds());
    if (priorPose.isEmpty()) {
      return Optional.empty();
    }

    var maybeFieldToTag = VisionConstants.kAprilTagLayout.getTagPose(poseEstimate.fiducialIds()[0]);
    if (maybeFieldToTag.isEmpty()) {
      return Optional.empty();
    }

    Pose2d fieldToTag =
        new Pose2d(maybeFieldToTag.get().toPose2d().getTranslation(), Rotation2d.kZero);

    Pose2d robotToTag = fieldToTag.relativeTo(poseEstimate.fieldToRobot());

    Pose2d posteriorPose =
        new Pose2d(
            fieldToTag
                .getTranslation()
                .minus(robotToTag.getTranslation().rotateBy(priorPose.get().getRotation())),
            priorPose.get().getRotation());

    double xStd = cam.standardDeviations[VisionConstants.kMegatag1XStdDevIndex];
    double yStd = cam.standardDeviations[VisionConstants.kMegatag1YStdDevIndex];
    double xyStd = Math.max(xStd, yStd);

    return Optional.of(
        new VisionPoseEstimateInField(
            posteriorPose,
            poseEstimate.timestampSeconds(),
            VecBuilder.fill(xyStd, xyStd, VisionConstants.kLargeVariance),
            poseEstimate.fiducialIds().length));
  }

  private Optional<VisionPoseEstimateInField> processMegatagPoseEstimate(
      MegaTagPoseEstimate poseEstimate,
      VisionIO.VisionIOInputs.CameraInputs cam,
      String logPrefix) {

    if (poseEstimate.timestampSeconds() <= state.lastMegatagTimestamp()) {
      return Optional.empty();
    }

    // Single‑tag extra checks
    if (poseEstimate.fiducialIds().length < 2) {

      if (cam.fiducialAprilTagObservation == null) return Optional.empty();

      for (var fiducial : cam.fiducialAprilTagObservation) {
        if (fiducial.ambiguity() > VisionConstants.kDefaultAmbiguityThreshold) {
          return Optional.empty();
        }
      }

      if (poseEstimate.avgTagArea() < VisionConstants.kTagMinAreaForSingleTagMegatag) {
        return Optional.empty();
      }

      var priorPose = state.getFieldToRobot(poseEstimate.timestampSeconds());
      if (poseEstimate.avgTagArea() < VisionConstants.kTagAreaThresholdForYawCheck
          && priorPose.isPresent()) {
        double yawDiff =
            Math.abs(
                MathUtil.angleModulus(
                    priorPose.get().getRotation().getRadians()
                        - poseEstimate.fieldToRobot().getRotation().getRadians()));

        if (yawDiff > Units.degreesToRadians(VisionConstants.kDefaultYawDiffThreshold)) {
          return Optional.empty();
        }
      }
    }

    if (poseEstimate.fieldToRobot().getTranslation().getNorm()
        < VisionConstants.kDefaultNormThreshold) {
      return Optional.empty();
    }

    if (cam.pose3d == null || Math.abs(cam.pose3d.getZ()) > VisionConstants.kDefaultZThreshold) {
      return Optional.empty();
    }

    // Exclusive‑tag filtering
    var exclusiveTag = state.getExclusiveTag();
    boolean hasExclusiveId =
        exclusiveTag.isPresent()
            && java.util.Arrays.stream(poseEstimate.fiducialIds())
                .anyMatch(id -> id == exclusiveTag.get());

    if (exclusiveTag.isPresent() && !hasExclusiveId) {
      return Optional.empty();
    }

    var loggedPose = state.getFieldToRobot(poseEstimate.timestampSeconds());
    if (loggedPose.isEmpty()) {
      return Optional.empty();
    }

    Pose2d estimatePose = poseEstimate.fieldToRobot();

    double quality = Math.max(poseEstimate.quality(), 0.001);
    double scaleFactor = 1.0 / quality;
    double xStd = cam.standardDeviations[VisionConstants.kMegatag1XStdDevIndex] * scaleFactor;
    double yStd = cam.standardDeviations[VisionConstants.kMegatag1YStdDevIndex] * scaleFactor;
    double rotStd = cam.standardDeviations[VisionConstants.kMegatag1YawStdDevIndex] * scaleFactor;

    double xyStd = Math.max(xStd, yStd);
    Matrix<N3, N1> visionStdDevs = VecBuilder.fill(xyStd, xyStd, rotStd);

    return Optional.of(
        new VisionPoseEstimateInField(
            estimatePose,
            poseEstimate.timestampSeconds(),
            visionStdDevs,
            poseEstimate.fiducialIds().length));
  }

  public void setUseVision(boolean useVision) {
    this.useVision = useVision;
  }
}
