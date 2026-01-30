package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterRollerIO {

  @AutoLog
  public class ShooterRollerIOInputs {
    public boolean motorConnected;
    public double appliedVolts;
    public double tempCelcius;
    public double positionIntake;
  }

  void updateInputs(ShooterRollerIOInputs inputs);

  void setVoltage(double voltage);

  void stopRollers();
}
