package frc.robot.subsystems.shooter.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterRollersIO {

  @AutoLog
  public class ShooterRollersIOInputs {
    public boolean motorConnected;
    public double appliedVolts;
    public double tempCelcius;
    public double velocity;
    public double acceleration;
  }

  void updateInputs(ShooterRollersIOInputs inputs);

  void setVoltage(double voltage);

  void stopRollers();
}
