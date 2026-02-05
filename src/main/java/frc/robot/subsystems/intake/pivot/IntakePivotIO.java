package frc.robot.subsystems.intake.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO {
  @AutoLog
  public class IntakePivotIOInputs {
    public boolean motorConnected;
    public double appliedVolts;
    public double tempCelcius;
    public double positionIntake;
  }

  void updateInputs(IntakePivotIOInputs inputs);

  void setVoltage(double voltage);

  void stopMotor();
}
