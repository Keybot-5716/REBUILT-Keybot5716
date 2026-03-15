package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class FieldConstants {

  private static final Pose2d STARTING_POSE = new Pose2d(3.1, 4, new Rotation2d());
  private static final Pose2d HUB_SHOOTING = new Pose2d(4.778, 4.293, new Rotation2d());

  public static boolean isRedAlliance() {
    return Robot.alliance.isPresent() && Robot.alliance.get() == DriverStation.Alliance.Red;
  }

  public static final double FIELD_LENGTH = 16.517589;
  public static final double FIELD_WIDTH = 8.049031;

  private static Pose2d toRed(Pose2d bluePose) {
    return new Pose2d(
        FIELD_LENGTH - bluePose.getX(),
        FIELD_WIDTH - bluePose.getY(),
        bluePose.getRotation().plus(Rotation2d.k180deg));
  }

  public static Pose2d allianceFlip(Pose2d pose) {
    if (isRedAlliance()) {
      return toRed(pose);
    }
    return pose;
  }

  public static Pose2d getTestingPose() {
    return allianceFlip(STARTING_POSE);
  }

  public static Pose2d getHubShootingPose() {
    return allianceFlip(HUB_SHOOTING);
  }
}
