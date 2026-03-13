package frc.robot.subsystems.intake.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollersIO {

  @AutoLog
  public class IntakeRollersIOInputs {
    public boolean motorConnected = false;
    public double appliedVolts = 0.0;
    public double tempCelcius = 0.0;
  }

  void updateInputs(IntakeRollersIOInputs inputs);

  void setVoltage(double voltage);

  void stopRollers();
}
