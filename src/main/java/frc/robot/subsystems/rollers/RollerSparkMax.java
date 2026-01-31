package frc.robot.subsystems.rollers;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;

public class RollerSparkMax implements RollerIO {

  private SparkMax motor;

  private SparkMaxConfig config = new SparkMaxConfig();

  private double acceleration;
  private double appliedVolts;
  private double tempCelsius;

  public RollerSparkMax(int deviceID) {
    motor = new SparkMax(deviceID, MotorType.kBrushless);

    config.smartCurrentLimit(40).idleMode(IdleMode.kBrake);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.motorConnected = true;
    inputs.acceleration = motor.getAppliedOutput();
    inputs.appliedVolts = motor.getAppliedOutput();
    inputs.tempCelcius = motor.getMotorTemperature();
  }

  @Override
  public void setVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -12, 12);
    motor.setVoltage(voltage);
  }

  @Override
  public void stopRoller() {
    motor.stopMotor();
  }
}
