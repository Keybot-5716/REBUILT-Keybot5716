package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterHoodIOTalonFX implements ShooterHoodIO {

  private final TalonFX motor;
  private TalonFXConfiguration config = new TalonFXConfiguration();

  private final VoltageOut voltageOutRequest = new VoltageOut(0);
  private final PositionVoltage positionRequest = new PositionVoltage(0);
  private final MotionMagicExpoVoltage motionMagicEVRequest = new MotionMagicExpoVoltage(0);

  public ShooterHoodIOTalonFX() {

    motor = new TalonFX(31, new CANBus("canivore"));

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 90;

    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 30;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.Slot0.kP = 7.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;

    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.data =
        new HoodIOData(
            BaseStatusSignal.isAllGood(
                motor.getPosition(),
                motor.getVelocity(),
                motor.getMotorVoltage(),
                motor.getSupplyCurrent(),
                motor.getDeviceTemp()),
            motor.getPosition().getValueAsDouble(),
            motor.getVelocity().getValueAsDouble(),
            motor.getMotorVoltage().getValueAsDouble(),
            motor.getSupplyCurrent().getValueAsDouble(),
            motor.getDeviceTemp().getValueAsDouble());
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void runOpenLoop(double output) {
    motor.setControl(new DutyCycleOut(output));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageOutRequest.withOutput(volts));
  }

  @Override
  public void setPosition(double position) {
    motor.setControl(positionRequest.withPosition(position));
  }

  @Override
  public void setNeutralModeBreak(boolean enable) {
    config.MotorOutput.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void resetEncoder() {
    motor.setPosition(0);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;

    motor.getConfigurator().apply(config);
  }
}
