package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.LoggedTunableNumber;
import frc.lib.util.DataProcessor;
import org.littletonrobotics.junction.Logger;

public class IntakePivotSubsystem extends SubsystemBase {
  private final IntakePivotIO pivotIO;

  private final IntakePivotIOInputsAutoLogged pivotInputs = new IntakePivotIOInputsAutoLogged();

  private TrapezoidProfile.Constraints profileConstraints =
      new TrapezoidProfile.Constraints(2.0, 2.0);

  private ProfiledPIDController controller =
      new ProfiledPIDController(4, 0, 0.08, profileConstraints);

  private TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);

  private static final LoggedTunableNumber pivotVolts =
      new LoggedTunableNumber("Intake/Pivot/PivotVolts", 2.0);

  private IntakeState intakeState = IntakeState.STOPPING_PIVOT;
  private DesiredState desiredState = DesiredState.STOPPPED_PIVOT;

  public enum DesiredState {
    FORWARD_PIVOT,
    REVERSE_PIVOT,
    STOPPPED_PIVOT,
    IN,
    OUT,
    TEST,
    TEST2
  }

  private enum IntakeState {
    FORWARDING_PIVOT,
    REVERSING_PIVOT,
    STOPPING_PIVOT,
    INING,
    OUTING,
    TEST,
    TEST2
  }

  public IntakePivotSubsystem(IntakePivotIO pivotIO) {
    this.pivotIO = pivotIO;
    controller.setTolerance(0.02);
    DataProcessor.initDataProcessor(
        () -> {
          synchronized (pivotInputs) {
            pivotIO.updateInputs(pivotInputs);
          }
        },
        pivotIO);
  }

  @Override
  public void periodic() {
    synchronized (pivotInputs) {
      Logger.processInputs("Intake/Pivot/PivotInputs", pivotInputs);

      Logger.recordOutput("Intake/Pivot/DesiredState", desiredState);
      Logger.recordOutput("Intake/Pivot/CurrentState", intakeState);
      intakeState = setStateTransition();
      applyStates();
    }
  }

  private IntakeState setStateTransition() {
    return switch (desiredState) {
      case FORWARD_PIVOT -> IntakeState.FORWARDING_PIVOT;
      case REVERSE_PIVOT -> IntakeState.REVERSING_PIVOT;
      case STOPPPED_PIVOT -> IntakeState.STOPPING_PIVOT;
      case IN -> IntakeState.INING;
      case OUT -> IntakeState.OUTING;
      case TEST -> IntakeState.TEST;
      case TEST2 -> IntakeState.TEST2;
    };
  }

  private void applyStates() {
    switch (intakeState) {
      case INING:
        setPosition(0.0);
        break;

      case OUTING:
        setPosition(0.0);
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

      case TEST:
        test(0.0);
        break;

      case TEST2:
        test(0.0);
        break;
    }
  }

  public void test(double position) {
    pivotIO.setPosition(position);
  }

  public void setPosition(double position) {
    goal = new TrapezoidProfile.State(position, 0);
    controller.setGoal(goal);
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
}
