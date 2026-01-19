package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {

  @AutoLog
  public class RollerIOInputs {
    public boolean motorConnected;
    public double acceleration;
    public double appliedVolts;
    public double tempCelcius;
    // Second motor inputs
    public boolean motorConnected2;
    public double acceleration2;
    public double appliedVolts2;
    public double tempCelcius2;
  }

  void updateInputs(RollerIOInputs inputs);

  void setVoltage(double voltage);

  void stopRoller();
}
