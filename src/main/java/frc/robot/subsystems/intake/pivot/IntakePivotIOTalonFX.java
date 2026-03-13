package frc.robot.subsystems.intake.pivot;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.superstructure.SuperstructureConstants.IDs;

public class IntakePivotIOTalonFX implements IntakePivotIO {

  protected TalonFX motor;
  private final VoltageOut voltageOut = new VoltageOut(Volts.zero());

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final StatusSignal<Angle> positionIntake;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Temperature> tempCelsius;

  public IntakePivotIOTalonFX() {
    // Cambiar el ID del motor
    motor = new TalonFX(IDs.INTAKE_PIVOT_ID, new CANBus("canivore"));

    /*config.Slot0.kP = 5;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0; */

    config.MotorOutput.Inverted =
        InvertedValue.CounterClockwise_Positive; // Cambiar por si es al revés
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Math.PI;

    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 30;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;
    config.CurrentLimits.SupplyCurrentLimit = 40;

    config.Feedback.SensorToMechanismRatio = 5;

    positionIntake = motor.getPosition();
    appliedVolts = motor.getMotorVoltage();
    tempCelsius = motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, positionIntake, appliedVolts, tempCelsius);

    motor.optimizeBusUtilization();
    motor.setPosition(0.0);
  }

  public void setPosition(double position) {
    motor.setPosition(position);
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public void stopMotor() {
    motor.stopMotor();
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    inputs.motorConnected = BaseStatusSignal.isAllGood(positionIntake, appliedVolts, tempCelsius);
    inputs.positionIntake = positionIntake.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.tempCelcius = tempCelsius.getValueAsDouble();
  }
}
