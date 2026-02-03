package frc.robot.subsystems.intake.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollersIO {

  @AutoLog
  public class IntakeRollersIOInputs {
    public boolean motorConnected;
    public double appliedVolts;
    public double tempCelcius;
  }

  void updateInputs(IntakeRollersIOInputs inputs);

  void setVoltage(double voltage);

  void stopRollers();
}
