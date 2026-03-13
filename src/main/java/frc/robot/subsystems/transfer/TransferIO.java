package frc.robot.subsystems.transfer;

import org.littletonrobotics.junction.AutoLog;

public interface TransferIO {

  @AutoLog
  public class TransferIOInputs {
    public boolean motorConnected = false;
    public double appliedVolts = 0.0;
    public double tempCelcius = 0.0;
  }

  void updateInputs(TransferIOInputs inputs);

  void setVoltage(double voltage);

  void stopRollers();
}
