package frc.robot.subsystems.vision.Objects;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {
  private final VisionIO io;
  private final RobotState state;
  private final VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();

  public VisionSubsystem(VisionIO io, RobotState state) {
    this.io = io;
    this.state = state;
  }

  @Override
  public void periodic() {
    io.readInputs(inputs);
    Logger.processInputs("Vision/Objects", inputs);

    // Llamamos a la función de procesamiento
    processFuels(inputs.cameraA);
  }

  private void processFuels(VisionIO.VisionIOInputs.CameraInputs cam) {
    if (!cam.seesTarget) {
      cam.objFuels.fuelPoseEstimate = null;
      return;
    }

    // 1. Obtener la pose del robot (si no hay, usamos una por defecto para evitar crash)
    var latestRobotPoseEntry = state.getLatestFieldToRobot();
    var robotPose = (latestRobotPoseEntry != null) ? latestRobotPoseEntry.getValue() : edu.wpi.first.math.geometry.Pose2d.kZero;

    // 2. Calcular la estimación (Trigonometría interna en el record)
    cam.objFuels.fuelPoseEstimate = FuelPoseEstimate.estimate(
        robotPose, 
        cam.tx, 
        cam.ty, 
        cam.ta, 
        Timer.getFPGATimestamp()
    );

    // 3. Telemetría y Logs
    FuelPoseEstimate estimate = cam.objFuels.fuelPoseEstimate;
    if (estimate != null) {
      // Log del objeto completo (Struct)
      Logger.recordOutput("Vision/Objects/FuelEstimate", FuelPoseEstimate.struct, estimate);
      
      // Log de la posición en el campo (Pose2d)
      Logger.recordOutput("Vision/Objects/FuelFieldPose", estimate.fieldToFuel());
      
      // Log de distancia para validar la trigonometría ty
      Logger.recordOutput("Vision/Objects/DistanceMeters", estimate.distanceToCamera());
    }
  }
}