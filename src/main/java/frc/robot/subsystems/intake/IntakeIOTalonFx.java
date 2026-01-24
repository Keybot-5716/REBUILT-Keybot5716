package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOTalonFx implements IntakeIO {
  protected TalonFX motor;
  private final VoltageOut voltageOut = new VoltageOut(Volts.zero());

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final StatusSignal<Angle> positionIntake;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Temperature> tempCelsius;

  public IntakeIOTalonFx() {
    // Cambiar el ID del motor
    motor = new TalonFX(16);

    config.Slot0.kP = 5;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;

    positionIntake = motor.getPosition();
    appliedVolts = motor.getMotorVoltage();
    tempCelsius = motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, positionIntake, appliedVolts, tempCelsius);

    motor.optimizeBusUtilization();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setControl(voltageOut.withOutput(voltage));
  }

  public void setPosition(double position) {
    motor.setPosition(position);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.motorConnected = BaseStatusSignal.isAllGood(positionIntake, appliedVolts, tempCelsius);
    inputs.positionIntake = positionIntake.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.tempCelcius = tempCelsius.getValueAsDouble();
  }
}
