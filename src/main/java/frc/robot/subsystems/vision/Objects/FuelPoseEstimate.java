package frc.robot.subsystems.vision.Objects;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

public record FuelPoseEstimate(
    Pose2d fieldToRobot,
    Pose2d fieldToFuel,
    double distanceToCamera,
    double timestampSeconds,
    double avgTagArea
) implements StructSerializable {

  public static FuelPoseEstimate estimate(Pose2d robotPose, double tx, double ta, double timestamp) {
    double k = 1.5; 
    double distance = (ta > 0) ? (k / Math.sqrt(ta)) : 0;

    Rotation2d angleToFuel = robotPose.getRotation().plus(Rotation2d.fromDegrees(-tx));
    
    Translation2d fuelTranslation = new Translation2d(
        distance * angleToFuel.getCos(),
        distance * angleToFuel.getSin()
    );

    Pose2d fieldToFuel = new Pose2d(
        robotPose.getTranslation().plus(fuelTranslation),
        angleToFuel
    );

    return new FuelPoseEstimate(robotPose, fieldToFuel, distance, timestamp, ta);
  }

  public static final FuelPoseEstimateStruct struct = new FuelPoseEstimateStruct();

  public static class FuelPoseEstimateStruct implements Struct<FuelPoseEstimate> {
    @Override public Class<FuelPoseEstimate> getTypeClass() { return FuelPoseEstimate.class; }
    @Override public String getTypeString() { return "record:FuelPoseEstimate"; }
    @Override public int getSize() { return (Pose2d.struct.getSize() * 2) + (3 * Double.BYTES); }
    @Override public String getSchema() { 
        return "Pose2d fieldToRobot;Pose2d fieldToFuel;double distanceToCamera;double timestampSeconds;double avgTagArea"; 
    }
    @Override public Struct<?>[] getNested() { return new Struct<?>[] {Pose2d.struct}; }

    @Override
    public FuelPoseEstimate unpack(ByteBuffer bb) {
      return new FuelPoseEstimate(
          Pose2d.struct.unpack(bb), 
          Pose2d.struct.unpack(bb), 
          bb.getDouble(), 
          bb.getDouble(), 
          bb.getDouble()
      );
    }

    @Override
    public void pack(ByteBuffer bb, FuelPoseEstimate value) {
      Pose2d.struct.pack(bb, value.fieldToRobot());
      Pose2d.struct.pack(bb, value.fieldToFuel());
      bb.putDouble(value.distanceToCamera());
      bb.putDouble(value.timestampSeconds());
      bb.putDouble(value.avgTagArea());
    }
    
    @Override public String getTypeName() { return "FuelPoseEstimate"; }
  }
}