package frc.robot;

import java.util.Map;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.team254.ConcurrentTimeInterpolatableBuffer;
import frc.robot.subsystems.vision.VisionPoseEstimateInField;

public class RobotState {
  public static final double BUFFER_TIME = 1.0;

  private final Consumer<VisionPoseEstimateInField> fieldEstimation;

  public RobotState(Consumer<VisionPoseEstimateInField> fieldEstimation) {
    this.fieldEstimation = fieldEstimation;

    fieldToRobot.addSample(0.0, Pose2d.kZero);
  }

  private final ConcurrentTimeInterpolatableBuffer<Pose2d> fieldToRobot =
      ConcurrentTimeInterpolatableBuffer.createBuffer(BUFFER_TIME);
  
  private final AtomicReference<ChassisSpeeds> measuredRobotRelativeChassisSpeeds = new AtomicReference<>(new ChassisSpeeds());
  private final AtomicReference<ChassisSpeeds> measuredFieldRelativeChassisSpeeds = new AtomicReference<>(new ChassisSpeeds());
  private final AtomicReference<ChassisSpeeds> desiredRobotRelativeChassisSpeeds = new AtomicReference<>(new ChassisSpeeds());
  private final AtomicReference<ChassisSpeeds> desiredFieldRelativeChassisSpeeds = new AtomicReference<>(new ChassisSpeeds());
  private final AtomicReference<ChassisSpeeds> fusedFieldRelativeChassisSpeeds = new AtomicReference<>(new ChassisSpeeds());

  private final ConcurrentTimeInterpolatableBuffer<Double> drivePitchAngularVelocity = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(BUFFER_TIME);
  private final ConcurrentTimeInterpolatableBuffer<Double> driveRollAngularVelocity = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(BUFFER_TIME);
  private final ConcurrentTimeInterpolatableBuffer<Double> driveYawAngularVelocity = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(BUFFER_TIME);

  private final ConcurrentTimeInterpolatableBuffer<Double> drivePitchRads = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(BUFFER_TIME);
  private final ConcurrentTimeInterpolatableBuffer<Double> driveRollRads = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(BUFFER_TIME);
  private final ConcurrentTimeInterpolatableBuffer<Double> accelX = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(BUFFER_TIME);
  private final ConcurrentTimeInterpolatableBuffer<Double> accelY = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(BUFFER_TIME);

  private final AtomicInteger iteration = new AtomicInteger(0);

  public void addOdometryTimeMeasurement(double timestamp, Pose2d pose) {
    fieldToRobot.addSample(timestamp, pose);
  }

  public void incrementIteration() {
    iteration.incrementAndGet();
  }

  public void addSwerveDriveMotionMeasurement(
    double timestamp,
    double angularRollRadPS,
    double angularPitchRadPS,
    double angularYawRadPS,
    double pitchRad,
    double rollRad,
    double accelX,
    double accelY,
    ChassisSpeeds desiredRobotRelativeChassisSpeeds,
    ChassisSpeeds desiredFieldRelativeChassisSpeeds,
    ChassisSpeeds measuredSpeeds,
    ChassisSpeeds measuredFieldRelativeSpeeds,
    ChassisSpeeds fusedFieldRelativeSpeeds
  ) {
    this.driveRollAngularVelocity.addSample(timestamp, angularPitchRadPS);
    this.drivePitchAngularVelocity.addSample(timestamp, angularPitchRadPS);
    this.driveYawAngularVelocity.addSample(timestamp, angularYawRadPS);
    this.drivePitchRads.addSample(timestamp, pitchRad);
    this.driveRollRads.addSample(timestamp, rollRad);
    this.accelX.addSample(timestamp, accelX);
    this.accelY.addSample(timestamp, accelY);

    this.desiredRobotRelativeChassisSpeeds.set(desiredRobotRelativeChassisSpeeds);
    this.desiredFieldRelativeChassisSpeeds.set(desiredFieldRelativeChassisSpeeds);
    this.measuredRobotRelativeChassisSpeeds.set(measuredSpeeds);
    this.measuredFieldRelativeChassisSpeeds.set(measuredFieldRelativeSpeeds);
    this.fusedFieldRelativeChassisSpeeds.set(fusedFieldRelativeSpeeds);
  }

  public boolean isBlueAlliance() {
    return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().equals(Optional.of(Alliance.Blue));
  }

  public boolean isRedAlliance() {
    return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().equals(Optional.of(Alliance.Red));
  }

  public Map.Entry<Double, Pose2d> getLatestFieldToRobot() {
    return fieldToRobot.getLatest();
  }

  public ChassisSpeeds getLatestRobotRelativeChassisSpeeds() {
    return measuredRobotRelativeChassisSpeeds.get();
  }

  public Pose2d getPredictedFieldToRobot(double bufferTime) {
    var fieldToRobotPredicted = getLatestFieldToRobot();
    Pose2d fieldToRobot = (fieldToRobotPredicted == null) ? Pose2d.kZero : fieldToRobotPredicted.getValue();
    var delta = getLatestRobotRelativeChassisSpeeds();
    delta = delta.times(bufferTime);
    return fieldToRobot.exp(new Twist2d(delta.vxMetersPerSecond, delta.vyMetersPerSecond, delta.omegaRadiansPerSecond));
  }
}
