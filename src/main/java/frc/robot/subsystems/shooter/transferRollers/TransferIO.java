package frc.robot.subsystems.shooter.transferRollers;

import org.littletonrobotics.junction.AutoLog;

public interface TransferIO {

  @AutoLog
  public class ShooterRollersIOInputs {
    public boolean motorConnected;
    public double appliedVolts;
    public double tempCelcius;
  }

  void updateInputs(ShooterRollersIOInputs inputs);

  void setVoltage(double voltage);

  void stopRollers();
}
