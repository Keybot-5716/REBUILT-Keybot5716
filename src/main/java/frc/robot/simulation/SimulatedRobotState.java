package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.intake.pivot.IntakePivotIOSim;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

public class SimulatedRobotState {
  TimeInterpolatableBuffer<Pose2d> fieldToRobotSim =
      TimeInterpolatableBuffer.createBuffer(RobotState.BUFFER_TIME);

  private SwerveDriveSimulation simSwerveDrive;
  private IntakePivotIOSim intakePivotSim;
  private final RobotContainer container;

  public SimulatedRobotState(RobotContainer container) {
    this.container = container;
  }

  public void init() {
    this.simSwerveDrive = this.container.getDriveSubsystem().getMapleSimDrive().mapleSimDrive;
    this.intakePivotSim = this.container.getIntakePivotIOSim();
  }

  public synchronized void addFieldToRobot(Pose2d pose) {
    fieldToRobotSim.addSample(Timer.getFPGATimestamp(), pose);
  }

  public synchronized Pose2d getLatestFieldToRobot() {
    var entry = fieldToRobotSim.getInternalBuffer().lastEntry();
    if (entry == null) return null;
    return entry.getValue();
  }

  public synchronized void updateSim() {
    var pose = simSwerveDrive.getSimulatedDriveTrainPose();
    Logger.recordOutput("FieldSimulation/SimulatedPose", pose);
    Logger.recordOutput("Intake/Pivot/Sim/FuelInside", intakePivotSim.isFuelInsideIntake());
    Logger.recordOutput("Intake/Pivot/Sim/FuelAmount", intakePivotSim.getFuelAmount());
    Logger.recordOutput("Intake/Pivot/Sim/IsRunning", intakePivotSim.isRunning());
  }
}
