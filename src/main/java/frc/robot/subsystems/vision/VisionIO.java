package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
@AutoLog
class VisionIOInputs {
  public boolean ntConnected = false;
}
class AprilTagIOInputs {
  public FiducialAprilTagObservation fiducialAprilTagObservation [];
  
}
class ObjDetectVisionIOInputs{

}
}
