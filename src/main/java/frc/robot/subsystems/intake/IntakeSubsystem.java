package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.intake.pivot.IntakePivotIO;
import frc.robot.subsystems.intake.pivot.IntakePivotIOInputsAutoLogged;
import frc.robot.subsystems.intake.rollers.IntakeRollersIO;
import frc.robot.subsystems.intake.rollers.IntakeRollersIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  private final IntakePivotIO pivotIO;
  private final IntakeRollersIO rollersIO;

  // private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakePivotIOInputsAutoLogged pivotInputs = new IntakePivotIOInputsAutoLogged();
  private final IntakeRollersIOInputsAutoLogged rollersInputs =
      new IntakeRollersIOInputsAutoLogged();

  // Creamos un objeto TrapezoidProfile que tenga velocidad y aceleración máximas
  private TrapezoidProfile.Constraints profileConstraints =
      new TrapezoidProfile.Constraints(2.0, 2.0);
  private ProfiledPIDController controller =
      new ProfiledPIDController(4, 0, 0.08, profileConstraints);

  private static TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);
  private TrapezoidProfile profile = new TrapezoidProfile(profileConstraints);

  private State Position = new State(0, 0);

  private State desiredPoint = new State(0, 0);

  private static final LoggedTunableNumber rollerVolts =
      new LoggedTunableNumber("Intake/Rollers/RollerVoltsIntake", 5.0);

  private static final LoggedTunableNumber pivotVolts =
      new LoggedTunableNumber("Intake/Pivot/PivotVolts", 2.0);

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
  private IntakeState intakeState = IntakeState.STOPPING;

  private double desiredRollersVoltage;
  private double desiredPivotVoltageForOpenLoop;

  public enum DesiredState {
    STOPPED,
    FORWARD_ROLLERS,
    REVERSE_ROLLERS,
    FORWARD_PIVOT,
    REVERSE_PIVOT,
    STOPPPED_PIVOT,
    IN,
    OUT
  }

  private enum IntakeState {
    STOPPING,
    FORWARDING_ROLLERS,
    REVERSING_ROLLERS,
    FORWARDING_PIVOT,
    REVERSING_PIVOT,
    STOPPING_PIVOT,
    INING,
    OUTING
  }

  public IntakeSubsystem(IntakeRollersIO rollersIO, IntakePivotIO pivotIO) {
    this.rollersIO = rollersIO;
    this.pivotIO = pivotIO;
  }

  @Override
  public void periodic() {
    /*????
    if (DriverStation.isDisabled()) {
      controller.reset(inputs.positionIntake);
    }
      */
    // Logger.processInputs("IntakeInputs/IntakePivotInputs", pivotInputs);
    Logger.processInputs("IntakeInputs/IntakeRollersInputs", rollersInputs);

    Logger.processInputs("IntakeInputs/IntakePivotInputs", pivotInputs);

    // pivotIO.updateInputs(pivotInputs);
    rollersIO.updateInputs(rollersInputs);
    pivotIO.updateInputs(pivotInputs);

    // pivotDisconnected.set(!pivotDebouncer.calculate(pivotInputs.motorConnected));

    intakeState = setStateTransition();
    applyStates();
  }

  private IntakeState setStateTransition() {
    return switch (desiredState) {
      case STOPPED -> IntakeState.STOPPING;
      case FORWARD_ROLLERS -> IntakeState.FORWARDING_ROLLERS;
      case REVERSE_ROLLERS -> IntakeState.REVERSING_ROLLERS;
      case FORWARD_PIVOT -> IntakeState.FORWARDING_PIVOT;
      case REVERSE_PIVOT -> IntakeState.REVERSING_PIVOT;
      case STOPPPED_PIVOT -> IntakeState.STOPPING_PIVOT;
      case IN -> IntakeState.INING;
      case OUT -> IntakeState.OUTING;
    };
  }

  private void applyStates() {
    switch (intakeState) {
      case FORWARDING_ROLLERS:
        setVoltage(rollerVolts.get());
        break;

      case REVERSING_ROLLERS:
        setVoltage(-rollerVolts.get());
        break;

      case STOPPING:
        stop();
        break;

      case INING:
        setPosition(IntakeIOConstants.In);
        break;

      case OUTING:
        setPosition(IntakeIOConstants.Out);
        break;

      case FORWARDING_PIVOT:
        runPivotOL(pivotVolts.get());
        break;

      case REVERSING_PIVOT:
        runPivotOL(-pivotVolts.get());
        break;

      case STOPPING_PIVOT:
        stopPivot();
        break;
    }
  }

  public void setPosition(double position) {
    goal.position = position;

    desiredPoint.position = position;
    Position.position = pivotInputs.positionIntake;

    profile.calculate(2.0, Position, desiredPoint);
    pivotIO.setVoltage(controller.calculate(pivotInputs.positionIntake, goal.position));
  }

  public void stop() {
    // pivotIO.stopMotor();
    rollersIO.stopRollers();
  }

  public void setVoltage(double voltage) {
    rollersIO.setVoltage(voltage);
  }

  public void runPivotOL(double voltage) {
    pivotIO.setVoltage(voltage);
  }

  public void stopPivot() {
    pivotIO.stopMotor();
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
