package frc.robot.subsystems.shooter.rollers;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ShooterRollersTalonFX implements ShooterRollersIO {
  protected TalonFX motor;
  private final VoltageOut voltageOut = new VoltageOut(Volts.zero());

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Temperature> tempCelsius;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<AngularAcceleration> acceleration;

  public ShooterRollersTalonFX() {
    // Cambiar el ID del motor
    motor = new TalonFX(17);

    config.Slot0.kP = 5;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;

    appliedVolts = motor.getMotorVoltage();
    tempCelsius = motor.getDeviceTemp();
    velocity = motor.getVelocity();
    acceleration = motor.getAcceleration();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, appliedVolts, tempCelsius, velocity, acceleration);

    motor.optimizeBusUtilization();
  }

  public void setPosition(double position) {
    motor.setPosition(position);
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public void stopRollers() {
    motor.stopMotor();
  }

  @Override
  public void updateInputs(ShooterRollersIOInputs inputs) {
    inputs.motorConnected =
        BaseStatusSignal.isAllGood(appliedVolts, tempCelsius, velocity, acceleration);
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.tempCelcius = tempCelsius.getValueAsDouble();
    inputs.velocity = velocity.getValueAsDouble();
    inputs.acceleration = acceleration.getValueAsDouble();
  }
}
