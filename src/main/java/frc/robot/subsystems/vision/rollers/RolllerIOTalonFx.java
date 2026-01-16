package frc.robot.subsystems.vision.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class RolllerIOTalonFx implements RollerIO{
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

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
    public void updateInputs(RollerIOInputs inputs){
        inputs.motorConnected =
            BaseStatusSignal.isAllGood(
                acceleration,
                appliedVolts,
                tempCelsius
            );
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
