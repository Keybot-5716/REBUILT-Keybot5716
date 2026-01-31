package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  private final IntakeIO io;

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

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

  private double desiredVoltage;

  public enum DesiredState {
    STOPPED,
    FORWARD,
    REVERSE,
    IN,
    OUT
  }

  private enum IntakeState {
    STOPPING,
    FORWARDING,
    REVERSING,
    INING,
    OUTING
  }

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    /*????
    if (DriverStation.isDisabled()) {
      controller.reset(inputs.positionIntake);
    }
      */

    Logger.processInputs("intakeInputs", inputs);
    io.updateInputs(inputs);

    intakeState = setStateTransition();
    applyStates();
  }

  private IntakeState setStateTransition() {
    return switch (desiredState) {
      case STOPPED -> IntakeState.STOPPING;
      case FORWARD -> IntakeState.FORWARDING;
      case REVERSE -> IntakeState.REVERSING;
      case IN -> IntakeState.INING;
      case OUT -> IntakeState.OUTING;
    };
  }

  public void stop() {
    io.stopRollers();
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  private void applyStates() {
    switch (intakeState) {
      case FORWARDING:
        setVoltage(desiredVoltage);
        break;

      case REVERSING:
        setVoltage(-desiredVoltage);
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
    }
  }

  public void setPosition(double position) {
    goal.position = position;
    io.setVoltage(controller.calculate(inputs.positionIntake, goal.position));
  }

  public void setDesiredState(DesiredState desiredState) {
    this.desiredState = desiredState;
  }

  public void setDesiredStateWithVoltage(DesiredState desiredState, double desiredVoltage) {
    this.desiredState = desiredState;
    this.desiredVoltage = desiredVoltage;
  }
}
