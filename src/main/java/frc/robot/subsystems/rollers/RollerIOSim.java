package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.Logger;

public class RollerIOSim extends RolllerIOTalonFx {
  public RollerIOSim() {
    super();
    Logger.recordOutput("Rollers/Sim/VoltageRequested", 0.0);
  }
}
