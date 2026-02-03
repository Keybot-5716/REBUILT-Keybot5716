package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.pivot.IntakePivotIO;
import frc.robot.subsystems.intake.pivot.IntakePivotIOInputsAutoLogged;
import frc.robot.subsystems.intake.rollers.IntakeRollersIO;
import frc.robot.subsystems.intake.rollers.IntakeRollersIOInputsAutoLogged;

public class IntakeSubsystem extends SubsystemBase {
  private final IntakePivotIO pivotIO;
  private final IntakeRollersIO rollersIO;

  // private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakePivotIOInputsAutoLogged pivotInputs = new IntakePivotIOInputsAutoLogged();
  private final IntakeRollersIOInputsAutoLogged rollersInputs = new IntakeRollersIOInputsAutoLogged();

  // private static final LoggedTunableNumber rollerVolts = new
  // LoggedTunableNumber("Intake/Rollers/RollerVolts", 7.0);

  // Creamos un objeto TrapezoidProfile que tenga velocidad y aceleración máximas
  private TrapezoidProfile.Constraints profile = new TrapezoidProfile.Constraints(2.0, 2.0);

  // Creamos un controller para el trapezoidProfile
  private ProfiledPIDController controller = new ProfiledPIDController(5, 0, 0, profile);

  // Cambiar hasta qué posición queremos que llegue el intake
  private static TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);

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
    IN,
    OUT
  }

  private enum IntakeState {
    STOPPING,
    FORWARDING_ROLLERS,
    REVERSING_ROLLERS,
    FORWARDING_PIVOT,
    REVERSING_PIVOT,
    INING,
    OUTING
  }

  public IntakeSubsystem(IntakePivotIO pivotIO, IntakeRollersIO rollersIO) {
    this.pivotIO = pivotIO;
    this.rollersIO = rollersIO;
  }

  @Override
  public void periodic() {
    /*????
    if (DriverStation.isDisabled()) {
      controller.reset(inputs.positionIntake);
    }
      */
    Logger.processInputs("Intake/IntakePivotInputs", pivotInputs);
    Logger.processInputs("Intake/IntakeRollersInputs", rollersInputs);

    pivotIO.updateInputs(pivotInputs);
    rollersIO.updateInputs(rollersInputs);

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
      case IN -> IntakeState.INING;
      case OUT -> IntakeState.OUTING;
    };
  }

  private void applyStates() {
    switch (intakeState) {
      case FORWARDING_ROLLERS:
        setVoltage(desiredRollersVoltage);
        break;

      case REVERSING_ROLLERS:
        setVoltage(-desiredRollersVoltage);
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
        runPivotOL(desiredPivotVoltageForOpenLoop);
        break;
      
      case REVERSING_PIVOT:
        runPivotOL(-desiredPivotVoltageForOpenLoop);
        break;
    }
  }

  public void setPosition(double position) {
    goal.position = position;
    pivotIO.setVoltage(controller.calculate(pivotInputs.positionIntake, goal.position));
  }

  public void stop() {
    pivotIO.stopMotor();
    rollersIO.stopRollers();
  }

  public void setVoltage(double voltage) {
    rollersIO.setVoltage(voltage);
  }

  public void runPivotOL(double voltage) {
    pivotIO.setVoltage(voltage);
  }

  public void setDesiredState(DesiredState desiredState) {
    this.desiredState = desiredState;
  }

  public void setDesiredStateWithVoltage(DesiredState desiredState, double desiredVoltage) {
    this.desiredState = desiredState;
    this.desiredRollersVoltage = desiredVoltage;
  }

}
