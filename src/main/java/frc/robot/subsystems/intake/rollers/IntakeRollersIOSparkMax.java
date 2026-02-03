package frc.robot.subsystems.intake.rollers;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;

public class IntakeRollersIOSparkMax implements IntakeRollersIO {
  private SparkMax motor;

  private SparkMaxConfig config = new SparkMaxConfig();

  private double acceleration;
  private double appliedVolts;
  private double tempCelsius;

  public IntakeRollersIOSparkMax() {
    motor = new SparkMax(1, MotorType.kBrushless);

    config.smartCurrentLimit(40).idleMode(IdleMode.kBrake);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeRollersIOInputs inputs) {
    inputs.motorConnected = true;
    inputs.appliedVolts = motor.getAppliedOutput();
    inputs.tempCelcius = motor.getMotorTemperature();
  }

  @Override
  public void setVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -12, 12);
    motor.setVoltage(voltage);
  }

  @Override
  public void stopRollers() {
    motor.stopMotor();
  }
}
