package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class RollerSparkMax implements RollerIO {

  private SparkMax motor;

  private SparkMaxConfig config = new SparkMaxConfig();

  private StatusSignal<AngularAcceleration> acceleration;
  private StatusSignal<Voltage> appliedVolts;
  private StatusSignal<Temperature> tempCelsius;

  public RollerSparkMax() {
    motor = new SparkMax(0, null);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.motorConnected = BaseStatusSignal.isAllGood(acceleration, appliedVolts, tempCelsius);
    inputs.acceleration = acceleration.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.tempCelcius = tempCelsius.getValueAsDouble();
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
