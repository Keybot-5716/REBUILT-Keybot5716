package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public class IntakeIOInputs {
    public boolean motorConnected;
    public double appliedVolts;
    public double tempCelcius;
    public double positionIntake;
  }

  void updateInputs(IntakeIOInputs inputs);

  void setVoltage(double voltage);

  void stopRollers();
}
