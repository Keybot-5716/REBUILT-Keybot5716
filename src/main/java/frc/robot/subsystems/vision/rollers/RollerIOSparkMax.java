package frc.robot.subsystems.vision.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class RollerIOSparkMax implements RollerIO {

  private SparkMax motor;

  private SparkMaxConfig config = new SparkMaxConfig();

  private StatusSignal<AngularAcceleration> acceleration;
  private StatusSignal<Voltage> appliedVolts;
  private StatusSignal<Temperature> tempCelsius;

  public RollerIOSparkMax() {
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
  public void setRollerSpeed(double speed) {
    motor.set(speed);
  }

  @Override
  public void stopRoller() {
    motor.stopMotor();
  }

  @Override
  public void runOpenLoop(double output) {
    motor.set(output);
  }
}
