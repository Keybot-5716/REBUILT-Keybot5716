package frc.robot.subsystems.vision.Objects;

import edu.wpi.first.networktables.*;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.concurrent.atomic.AtomicReference;

public class VisionIOLimelight implements VisionIO {
  private final NetworkTable tableA =
      NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightATableName);

  private final AtomicReference<VisionIOInputs> latestInputs =
      new AtomicReference<>(new VisionIOInputs());

  public VisionIOLimelight(RobotState robotState) {
    setLLSettings();
  }

  /** Configura la posición de la cámara en el espacio del robot dentro de la Limelight. */
  private void setLLSettings() {
    double[] cameraAPose = {
      VisionConstants.kRobotToCameraAForward,
      VisionConstants.kRobotToCameraASide,
      VisionConstants.kCameraAHeightOffGroundMeters,
      0.0, // Roll
      VisionConstants.kCameraAPitchDegrees,
      VisionConstants.kCameraAYawOffset.getDegrees()
    };

    tableA.getEntry("camerapose_robotspace_set").setDoubleArray(cameraAPose);
  }

  @Override
  public void readInputs(VisionIOInputs inputs) {
    readCameraData(tableA, inputs.cameraA, VisionConstants.kLimelightATableName);
    latestInputs.set(inputs);
  }

  /** Lee los datos de detección de objetos (Fuel) desde NetworkTables. */
  private void readCameraData(
      NetworkTable table, VisionIOInputs.CameraInputs camera, String limelightName) {

    // tv: 1.0 si la Limelight ve un objeto según su pipeline actual
    camera.seesTarget = table.getEntry("tv").getDouble(0) == 1.0;

    if (!camera.seesTarget) {
      camera.objFuels.count = 0;
      camera.fiducialObjObservation = new FiducialObjObservation[0];
      camera.objFuels.fuelPoseEstimate = null;
      return;
    }

    try {
      // Extraemos los valores del "Best Target" (el objeto con más confianza)
      double tx = table.getEntry("tx").getDouble(0.0);
      double ty = table.getEntry("ty").getDouble(0.0);
      double ta = table.getEntry("ta").getDouble(0.0);

      // tid: ID de la clase detectada (ej. 0 para Fuel, 1 para otra pieza)
      int classID = (int) table.getEntry("tid").getInteger(0);

      // Llenamos el array de observaciones con el objetivo principal
      camera.objFuels.count = 1;
      camera.fiducialObjObservation =
          new FiducialObjObservation[] {
            new FiducialObjObservation(
                classID, tx, // txnc (usado para ángulo)
                ty, // tync
                ta)
          };

    } catch (Exception e) {
      // Limpieza de seguridad en caso de error en la red
      System.err.println("Error procesando NetworkTables de Limelight: " + e.getMessage());
      camera.objFuels.count = 0;
      camera.fiducialObjObservation = new FiducialObjObservation[0];
    }
  }
}
