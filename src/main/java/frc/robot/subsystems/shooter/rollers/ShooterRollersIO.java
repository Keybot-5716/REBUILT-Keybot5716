package frc.robot.subsystems.shooter.rollers;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.util.*;

public interface ShooterRollersIO extends DataProcessor.IODataRefresher {

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

  void setVelocity(double voltage);

  void stopRollers();
  
  @Override
  default void refreshData() {}
}
