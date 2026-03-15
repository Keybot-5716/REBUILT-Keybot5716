package frc.robot.subsystems.intake.rollers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.LoggedTunableNumber;
import frc.lib.util.DataProcessor;
import org.littletonrobotics.junction.Logger;

public class IntakeRollersSubsystem extends SubsystemBase {
  private final IntakeRollersIO rollersIO;

  private final IntakeRollersIOInputsAutoLogged rollersInputs =
      new IntakeRollersIOInputsAutoLogged();

  private static final LoggedTunableNumber rollerVolts =
      new LoggedTunableNumber("Intake/Rollers/RollerVoltsIntake", 25.0);

  private DesiredState desiredState = DesiredState.STOPPED;
  private RollersState rollersState = RollersState.STOPPING;

  public enum DesiredState {
    STOPPED,
    FORWARD_ROLLERS,
    REVERSE_ROLLERS,
  }

  private enum RollersState {
    STOPPING,
    FORWARDING_ROLLERS,
    REVERSING_ROLLERS,
  }

  public IntakeRollersSubsystem(IntakeRollersIO rollersIO) {
    this.rollersIO = rollersIO;
    DataProcessor.initDataProcessor(
        () -> {
          synchronized (rollersInputs) {
            rollersIO.updateInputs(rollersInputs);
          }
        },
        rollersIO);
  }

  @Override
  public void periodic() {
    synchronized (rollersInputs) {
      Logger.processInputs("Intake/Rollers/RollersInputs", rollersInputs);

      Logger.recordOutput("Intake/Rollers/DesiredState", desiredState);
      Logger.recordOutput("Intake/Rollers/CurrentState", rollersState);
      rollersState = setStateTransition();
      applyStates();
    }
  }

  private RollersState setStateTransition() {
    return switch (desiredState) {
      case STOPPED -> RollersState.STOPPING;
      case FORWARD_ROLLERS -> RollersState.FORWARDING_ROLLERS;
      case REVERSE_ROLLERS -> RollersState.REVERSING_ROLLERS;
    };
  }

  private void applyStates() {
    switch (rollersState) {
      case FORWARDING_ROLLERS:
        setVoltage(rollerVolts.get());
        break;

      case REVERSING_ROLLERS:
        setVoltage(-rollerVolts.get());
        break;

      case STOPPING:
        stop();
        break;
    }
  }

  public void stop() {
    rollersIO.stopMotor();
  }

  public void setVoltage(double voltage) {
    rollersIO.setVoltage(voltage);
  }

  public void setDesiredState(DesiredState desiredState) {
    this.desiredState = desiredState;
  }
}
