package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {

  @AutoLog
  public class RollerIOInputs {
    public boolean motorConnected;
    public double acceleration;
    public double appliedVolts;
    public double tempCelcius;
  }

  void updateInputs(RollerIOInputs inputs);

  void setVoltage(double voltage);

  void stopRoller();
}
