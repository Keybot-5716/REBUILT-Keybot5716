package frc.robot.subsystems.intake.rollers;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class IntakeRollersSubsystem extends SubsystemBase {
  private final IntakeRollersIO rollersIO;

  // private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeRollersIOInputsAutoLogged rollersInputs =
      new IntakeRollersIOInputsAutoLogged();

  private static final LoggedTunableNumber rollerVolts =
      new LoggedTunableNumber("Intake/Rollers/RollerVoltsIntake", 5.0);

  // private static final LoggedTunableNumber pivotP = new LoggedTunableNumber("Intake/Pivot/kP");
  // private static final LoggedTunableNumber pivotD = new LoggedTunableNumber("Intake/Pivot/kD");

  /*
  static {
    switch (Constants.currentMode) {
      case REAL -> {
        pivotP.initDefault(3);
        pivotD.initDefault(0.01);
      }
      case SIM -> {
        pivotP.initDefault(5);
        pivotD.initDefault(0.01);
      }
    }
  }*/

  private final Debouncer pivotDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  // private final Alert pivotDisconnected = new Alert("Pivot disconnected :(",
  // Alert.AlertType.kWarning);

  private DesiredState desiredState = DesiredState.STOPPED;
  private RollersState rollersState = RollersState.STOPPING;

  private double desiredRollersVoltage;
  private double desiredPivotVoltageForOpenLoop;

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
  }

  @Override
  public void periodic() {

    // if (DriverStation.isDisabled()) {
    // controller.reset(pivotInputs.positionIntake);
    /////////////////////////// }
    rollersIO.updateInputs(rollersInputs);
    Logger.processInputs("IntakeRollersInputs", rollersInputs);

    rollersState = setStateTransition();
    applyStates();
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
    // pivotIO.stopMotor();
    rollersIO.stopRollers();
  }

  public void setVoltage(double voltage) {
    rollersIO.setVoltage(voltage);
  }

  public void setDesiredState(DesiredState desiredState) {
    this.desiredState = desiredState;
  }

  public void setDesiredStateWithVoltage(DesiredState desiredState, double desiredVoltage) {
    this.desiredState = desiredState;
    this.desiredRollersVoltage = desiredVoltage;
    this.desiredPivotVoltageForOpenLoop = desiredVoltage;
  }
}
