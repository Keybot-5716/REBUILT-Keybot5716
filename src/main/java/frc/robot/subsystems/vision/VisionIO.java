package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  class VisionIOInputs {
    public boolean ntConnected = false;
  }

  class AprilTagVisionIOInputs {
    public boolean seesTarget;
    public FiducialAprilTagObservation fiducialAprilTagObservation[];
    public int megatagcount;
    public int megatag2count;
    public Pose3d pose3d;
    public double[] standardDeviations =
        new double[12]; // [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1Yaw, MT2x,
    // MT2y, MT2z, MT2roll, MT2pitch, MT2yaw]
  }

  class ObjDetectVisionIOInputs {
    public boolean seesTarget;
    public FiducialObjObservation fiducialObjObservation[];
    public int objcount;
    public Pose3d pose3d;
    public double[] standardDeviations =
        new double[12]; // [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1Yaw, MT2x,
    // MT2y, MT2z, MT2roll, MT2pitch, MT2yaw]
  }

  public default void updateInputs(
      VisionIOInputs inputs,
      AprilTagVisionIOInputs aprilTagInputs,
      ObjDetectVisionIOInputs objDetectInputs) {}
}
