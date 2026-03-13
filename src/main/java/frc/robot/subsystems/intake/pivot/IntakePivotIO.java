package frc.robot.subsystems.intake.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO {
  @AutoLog
  public class IntakePivotIOInputs {
    public boolean motorConnected = false;
    public double appliedVolts = 0.0;
    public double tempCelcius = 0.0;
    public double positionIntake = 0.0;
  }

  void updateInputs(IntakePivotIOInputs inputs);

  void setVoltage(double voltage);

  void stopMotor();

  void setPosition(double position);
}
