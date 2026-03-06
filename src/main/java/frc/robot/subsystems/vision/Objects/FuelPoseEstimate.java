package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.lib.limelight.LimelightHelpers;
import java.nio.ByteBuffer;

/**
 * Represents a robot pose estimate using multiple AprilTags (Megatag).
 *
 * @param fieldToRobot The estimated robot pose on the field
 * @param timestampSeconds The timestamp when this estimate was captured
 * @param latency Processing latency in seconds
 * @param avgTagArea Average area of detected tags
 * @param quality Quality score of the pose estimate (0-1)
 * @param fiducialIds IDs of fiducials used for this estimate
 */
public record FuelPoseEstimate(
    Pose2d fieldToRobot, double timestampSeconds, double latency, double avgTagArea)
    implements StructSerializable {

  public FuelPoseEstimate {
    if (fieldToRobot == null) {
      fieldToRobot = Pose2d.kZero;
    }
  }

  /** Converts a Limelight pose estimate to a MegatagPoseEstimate. */
  public static FuelPoseEstimate fromLimelight(LimelightHelpers.PoseEstimate poseEstimate) {
    Pose2d fieldToRobot = poseEstimate.pose;
    if (fieldToRobot == null) {
      fieldToRobot = Pose2d.kZero;
    }

    return new FuelPoseEstimate(
        fieldToRobot, poseEstimate.timestampSeconds, poseEstimate.latency, poseEstimate.avgTagArea);
  }

  public static final FuelPoseEstimateStruct struct = new FuelPoseEstimateStruct();

  public static class FuelPoseEstimateStruct implements Struct<FuelPoseEstimate> {

    @Override
    public Class<FuelPoseEstimate> getTypeClass() {
      return FuelPoseEstimate.class;
    }

    @Override
    public String getTypeString() {
      return "record:FuelPoseEstimate";
    }

    @Override
    public int getSize() {
      return Pose2d.struct.getSize() + 3 * Double.BYTES;
    }

    @Override
    public String getSchema() {
      return "Pose2d fieldToRobot; double timestampSeconds; double latency; double avgTagArea";
    }

    @Override
    public Struct<?>[] getNested() {
      return new Struct<?>[] {Pose2d.struct};
    }

    @Override
    public FuelPoseEstimate unpack(ByteBuffer bb) {
      Pose2d fieldToRobot = Pose2d.struct.unpack(bb);
      double timestampSeconds = bb.getDouble();
      double latency = bb.getDouble();
      double avgTagArea = bb.getDouble();
      return new FuelPoseEstimate(fieldToRobot, timestampSeconds, latency, avgTagArea);
    }

    @Override
    public void pack(ByteBuffer bb, FuelPoseEstimate value) {
      Pose2d.struct.pack(bb, value.fieldToRobot());
      bb.putDouble(value.timestampSeconds());
      bb.putDouble(value.latency());
      bb.putDouble(value.avgTagArea());
    }

    @Override
    public String getTypeName() {
      return "FuelPoseEstimate";
    }
  }
}
