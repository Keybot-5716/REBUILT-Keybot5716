package frc.robot.subsystems.vision.Objects;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {
  private final VisionIO io;
  private final RobotState state;
  private final VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();
  private boolean useVision = true;

  public VisionSubsystem(VisionIO io, RobotState state) {
    this.io = io;
    this.state = state;
  }

  @Override
  public void periodic() {
    double startTime = Timer.getFPGATimestamp();
    io.readInputs(inputs);

    // 1. Logs manuales para evitar problemas con AutoLog
    logCameraInputs("Vision/FuelCameraA", inputs.cameraA);

    // 2. Procesar la cámara usando tu array de fiduciales
    Optional<FuelPoseEstimate> maybeFuel = processCamera(inputs.cameraA, "FuelCameraA");

    if (!useVision || maybeFuel.isEmpty()) {
      Logger.recordOutput("Vision/Fuel/UsingVision", useVision);
      return;
    }

    // 3. Si es válido, actualizar el estado y guardar en el IO para telemetría
    FuelPoseEstimate acceptedFuel = maybeFuel.get();
    
    // Guardar el resultado procesado de vuelta en la estructura de tu IO
    inputs.cameraA.objFuels.fuelPoseEstimate = acceptedFuel;
    
    // Notificar al RobotState para que los comandos lo usen
    state.addFuelObservation(acceptedFuel);

    Logger.recordOutput("Vision/Fuel/UsingVision", true);
    Logger.recordOutput("Vision/Fuel/AcceptedPose", acceptedFuel.fieldToFuel());
    Logger.recordOutput("Vision/Fuel/LatencySec", Timer.getFPGATimestamp() - startTime);
  }

  /**
   * Procesa las observaciones crudas de la cámara para generar una pose en el campo.
   */
  private Optional<FuelPoseEstimate> processCamera(
      VisionIO.VisionIOInputs.CameraInputs cam, String label) {

    // Validar que el array de fiduciales que definiste en el IO tenga datos
    if (cam.fiducialObjObservation == null || cam.fiducialObjObservation.length == 0) {
      return Optional.empty();
    }

    // Tomar la detección principal (índice 0)
    var observation = cam.fiducialObjObservation[0];

    // Obtener la odometría actual del robot
    var latestRobotPose = state.getLatestFieldToRobot();
    if (latestRobotPose == null) {
      return Optional.empty();
    }

    // Crear estimación con txnc y area del record FiducialObjObservation
    FuelPoseEstimate estimate = FuelPoseEstimate.estimate(
        latestRobotPose.getValue(), 
        observation.txnc(), 
        observation.area(), 
        Timer.getFPGATimestamp()
    );

    return Optional.ofNullable(estimate);
  }

  /**
   * Logs básicos de diagnóstico.
   */
  private void logCameraInputs(String prefix, VisionIO.VisionIOInputs.CameraInputs cam) {
    boolean hasData = cam.fiducialObjObservation != null && cam.fiducialObjObservation.length > 0;
    Logger.recordOutput(prefix + "/SeesTarget", cam.seesTarget && hasData);

    if (hasData) {
      Logger.recordOutput(prefix + "/RawTX", cam.fiducialObjObservation[0].txnc());
      Logger.recordOutput(prefix + "/RawArea", cam.fiducialObjObservation[0].area());
    }
  }

  public void setUseVision(boolean useVision) {
    this.useVision = useVision;
  }
}