package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class RolllerIOTalonFx implements RollerIO {
  private final VoltageOut voltageOut = new VoltageOut(0);

  protected final TalonFX motor;
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final StatusSignal<AngularAcceleration> acceleration;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Temperature> tempCelsius;

  public RolllerIOTalonFx() {

    // CAMBIAR!!!!!
    motor = new TalonFX(0);

    motor.getConfigurator().apply(config);

    acceleration = motor.getAcceleration();
    appliedVolts = motor.getMotorVoltage();
    tempCelsius = motor.getDeviceTemp();
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
    motor.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public void stopRoller() {
    motor.stopMotor();
  }
}
