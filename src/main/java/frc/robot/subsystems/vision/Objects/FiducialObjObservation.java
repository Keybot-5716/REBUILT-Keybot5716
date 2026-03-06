package frc.robot.subsystems.vision;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.lib.limelight.LimelightHelpers;
import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.Objects;

public record FiducialObjObservation(double txnc, double tync, double ambiguity, double area)
    implements StructSerializable {

  /** Converts a Limelight raw fiducial to a FiducialObservation. */
  public static FiducialObjObservation fromLimelight(LimelightHelpers.RawFiducial fiducial) {
    if (fiducial == null) {
      return null;
    }
    return new FiducialObjObservation(
        fiducial.txnc, fiducial.tync, fiducial.ambiguity, fiducial.ta);
  }

  /** Converts an array of Limelight raw fiducials to FiducialObservation array. */
  public static FiducialObjObservation[] fromLimelight(LimelightHelpers.RawFiducial[] fiducials) {
    if (fiducials == null) {
      return new FiducialObjObservation[0];
    }
    return Arrays.stream(fiducials)
        .map(FiducialObjObservation::fromLimelight)
        .filter(Objects::nonNull)
        .toArray(FiducialObjObservation[]::new);
  }

  public static final Struct<FiducialObjObservation> struct =
      new Struct<FiducialObjObservation>() {
        @Override
        public Class<FiducialObjObservation> getTypeClass() {
          return FiducialObjObservation.class;
        }

        @Override
        public String getTypeString() {
          return "record:FiducialObservation";
        }

        @Override
        public int getSize() {
          return Integer.BYTES + 4 * Double.BYTES;
        }

        @Override
        public String getSchema() {
          return "int id;double txnc;double tync;double ambiguity";
        }

        @Override
        public FiducialObjObservation unpack(ByteBuffer bb) {
          double txnc = bb.getDouble();
          double tync = bb.getDouble();
          double ambiguity = bb.getDouble();
          double area = bb.getDouble();
          return new FiducialObjObservation(txnc, tync, ambiguity, area);
        }

        @Override
        public void pack(ByteBuffer bb, FiducialObjObservation value) {
          bb.putDouble(value.txnc());
          bb.putDouble(value.tync());
          bb.putDouble(value.ambiguity());
          bb.putDouble(value.area());
        }

        @Override
        public String getTypeName() {
          return "FiducialObservation";
        }
      };
}
