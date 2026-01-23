package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class RolllerIOTalonFx implements RollerIO {
  private final VoltageOut voltageOut = new VoltageOut(0);

  // protected final TalonFX motor;
  protected final TalonFX motor2;
  // private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final TalonFXConfiguration config2 = new TalonFXConfiguration();

  // private final StatusSignal<AngularAcceleration> acceleration;
  // private final StatusSignal<Voltage> appliedVolts;
  // private final StatusSignal<Temperature> tempCelsius;

  // Second motor signals
  private final StatusSignal<AngularAcceleration> acceleration2;
  private final StatusSignal<Voltage> appliedVolts2;
  private final StatusSignal<Temperature> tempCelsius2;

  public RolllerIOTalonFx() {

    // motor = new TalonFX(RollerConstants.talon1Id);
    motor2 = new TalonFX(RollerConstants.talon2Id);

    /*
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.CurrentLimits.SupplyCurrentLowerLimit = 30;
    config.CurrentLimits.SupplyCurrentLowerTime = 1.0;
    motor.getConfigurator().apply(config);*/

    config2.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config2.CurrentLimits.SupplyCurrentLimitEnable = true;
    config2.CurrentLimits.SupplyCurrentLimit = 60;
    config2.CurrentLimits.SupplyCurrentLowerLimit = 30;
    config2.CurrentLimits.SupplyCurrentLowerTime = 1.0;
    motor2.getConfigurator().apply(config2);

    /*acceleration = motor.getAcceleration();
    appliedVolts = motor.getMotorVoltage();
    tempCelsius = motor.getDeviceTemp();*/

    // Second motor signals
    acceleration2 = motor2.getAcceleration();
    appliedVolts2 = motor2.getMotorVoltage();
    tempCelsius2 = motor2.getDeviceTemp();
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    // inputs.motorConnected = BaseStatusSignal.isAllGood(acceleration, appliedVolts, tempCelsius);
    // inputs.acceleration = acceleration.getValueAsDouble();
    // inputs.appliedVolts = appliedVolts.getValueAsDouble();
    // inputs.tempCelcius = tempCelsius.getValueAsDouble();
    // Second motor inputs
    inputs.motorConnected2 = BaseStatusSignal.isAllGood(acceleration2, appliedVolts2, tempCelsius2);
    inputs.acceleration2 = acceleration2.getValueAsDouble();
    inputs.appliedVolts2 = appliedVolts2.getValueAsDouble();
    inputs.tempCelcius2 = tempCelsius2.getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -12, 12);
    // motor.setControl(voltageOut.withOutput(voltage));
    // Preguntar si el segund motor va igual o al reves
    // motor2.setControl(new Follower(motor.getDeviceID(), MotorAlignmentValue.Opposed));
    motor2.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public void stopRoller() {
    // motor.stopMotor();
    motor2.stopMotor();
  }
}
