package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.rollers.RollerIO.RollerIOInputs;

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
}
