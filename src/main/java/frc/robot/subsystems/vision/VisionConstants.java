package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {

  public static final double kLargeVariance = 1e6;

  public static final int kMegatag1XStdDevIndex = 0;
  public static final int kMegatag1YStdDevIndex = 1;
  public static final int kMegatag1YawStdDevIndex = 5;

  public static final int kMegatag2XStdDevIndex = 6;
  public static final int kMegatag2YStdDevIndex = 7;
  public static final int kMegatag2YawStdDevIndex = 11;
  public static final int kExpectedStdDevArrayLength = 12;

  public static final int kMinFiducialCount = 1;

  public static final AprilTagFieldLayout kAprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

  public static final double kCameraAPitchDegrees = 20.0;
  public static final double kCameraAPitchRads = Units.degreesToRadians(kCameraAPitchDegrees);
  public static final double kCameraAHeightOffGroundMeters = 0.0;
  public static final String kLimelightATableName = "limelight-uno";
  public static final double kRobotToCameraAForward =0.0;
  public static final double kRobotToCameraASide = 0.0;
  public static final Rotation2d kCameraAYawOffset = Rotation2d.fromDegrees(0.0);
  public static final Transform2d kRobotToCameraA =
      new Transform2d(
          new Translation2d(kRobotToCameraAForward, kRobotToCameraASide), kCameraAYawOffset);

  public static final double kCameraBPitchDegrees = 20.0;
  public static final double kCameraBPitchRads = Units.degreesToRadians(kCameraAPitchDegrees);
  public static final double kCameraBHeightOffGroundMeters =0.0;
  public static final String kLimelightBTableName = "limelight-dos";
  public static final double kRobotToCameraBForward = 0.0;
  public static final double kRobotToCameraBSide = 0.0;
  public static final Rotation2d kCameraBYawOffset = Rotation2d.fromDegrees(0.0);
  public static final Transform2d kRobotToCameraB =
      new Transform2d(
          new Translation2d(kRobotToCameraBForward, kRobotToCameraBSide), kCameraBYawOffset);

  public static final double kDefaultAmbiguityThreshold = 0.19;
  public static final double kDefaultYawDiffThreshold = 5.0;
  public static final double kTagAreaThresholdForYawCheck = 2.0;
  public static final double kTagMinAreaForSingleTagMegatag = 1.0;
  public static final double kDefaultZThreshold = 0.2;
  public static final double kDefaultNormThreshold = 1.0;
  public static final double kMinAmbiguityToFlip = 0.08;
}
