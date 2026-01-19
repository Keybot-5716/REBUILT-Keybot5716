
package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class RolllerIOTalonFx implements RollerIO {
  private final VoltageOut voltageOut = new VoltageOut(0);

  protected final TalonFX motor;
  protected final TalonFX motor2;
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final StatusSignal<AngularAcceleration> acceleration;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Temperature> tempCelsius;

  // Second motor signals
  private final StatusSignal<AngularAcceleration> acceleration2;
  private final StatusSignal<Voltage> appliedVolts2;
  private final StatusSignal<Temperature> tempCelsius2;

  public RolllerIOTalonFx() {

    // CAMBIAR!!!!!
    motor = new TalonFX(6);
    motor2 = new TalonFX(7);

    motor.getConfigurator().apply(config);
    motor2.getConfigurator().apply(config);

    acceleration = motor.getAcceleration();
    appliedVolts = motor.getMotorVoltage();
    tempCelsius = motor.getDeviceTemp();

    // Second motor signals
    acceleration2 = motor2.getAcceleration();
    appliedVolts2 = motor2.getMotorVoltage();
    tempCelsius2 = motor2.getDeviceTemp();
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.motorConnected = BaseStatusSignal.isAllGood(acceleration, appliedVolts, tempCelsius);
    inputs.acceleration = acceleration.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.tempCelcius = tempCelsius.getValueAsDouble();
    // Second motor inputs
    inputs.motorConnected2 = BaseStatusSignal.isAllGood(acceleration2, appliedVolts2, tempCelsius2);
    inputs.acceleration2 = acceleration2.getValueAsDouble();
    inputs.appliedVolts2 = appliedVolts2.getValueAsDouble();
    inputs.tempCelcius2 = tempCelsius2.getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -12, 12);
    motor.setControl(voltageOut.withOutput(voltage));
    motor2.setControl(voltageOut.withOutput(voltage));

    motor2.setControl(new Follower(motor.getDeviceID(), MotorAlignmentValue.Aligned));
  }

  @Override
  public void stopRoller() {
    motor.stopMotor();
    motor2.stopMotor();
  }

}
