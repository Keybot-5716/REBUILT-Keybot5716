package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  class VisionIOInputs {
    public boolean ntConnected = false;
  }

  class AprilTagIOInputs {
    public FiducialAprilTagObservation fiducialAprilTagObservation[];
  }

  class ObjDetectVisionIOInputs {}
}
