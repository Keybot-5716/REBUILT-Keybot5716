package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.shooter.hood.ShooterHoodIO;
import frc.robot.subsystems.shooter.rollers.ShooterRollersIO;
import frc.robot.subsystems.shooter.transferRollers.TransferIO;

public class ShooterSubsystem extends SubsystemBase {
  private final ShooterRollersIO rollersIO;
  private final ShooterHoodIO hoodIO;
  private final TransferIO transferIO;

  // private final ShooterRollerIOInputsAutoLogged inputs = new ShooterRollerIOInputsAutoLogged();

  private static final LoggedTunableNumber rollerVolts =
      new LoggedTunableNumber("Intake/Rollers/RollerVolts", 7.0);

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
    OUT,
    TEST
  }

  private enum IntakeState {
    STOPPING,
    FORWARDING,
    REVERSING,
    INING,
    OUTING,
    TESTING
  }

  public ShooterSubsystem(ShooterRollersIO rollersIO, ShooterHoodIO hoodIO, TransferIO transferIO) {
    this.rollersIO = rollersIO;
    this.hoodIO = hoodIO;
    this.transferIO = transferIO;
  }

  @Override
  public void periodic() {
    /*????
    if (DriverStation.isDisabled()) {
      controller.reset(inputs.positionIntake);
    }
      */

    // Logger.processInputs("IntakeInputs", inputs);
    // io.updateInputs(inputs);

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
      case TEST -> IntakeState.TESTING;
    };
  }

  public void stop() {
    // io.stopRollers();
  }

  public void setVoltageRollers(double voltage) {
    // io.setVoltage(voltage);
  }

  public void setVoltageIntakeTesters(double voltage) {
    // io.setVoltage(voltage);
  }

  private void applyStates() {
    switch (intakeState) {
      case FORWARDING:
        setVoltageRollers(rollerVolts.get());
        break;

      case REVERSING:
        setVoltageRollers(-rollerVolts.get());
        break;

      case STOPPING:
        setVoltageRollers(0);
        break;

      case INING:
        setPosition(ShooterConstants.In);
        break;

      case OUTING:
        setPosition(ShooterConstants.Out);
        break;

      case TESTING:
        setVoltageIntakeTesters(desiredVoltage);
        break;
    }
  }

  public void setPosition(double position) {
    goal.position = position;
    // io.setVoltage(controller.calculate(inputs.positionIntake, goal.position));
  }

  public void setDesiredState(DesiredState desiredState) {
    this.desiredState = desiredState;
  }

  public void setDesiredStateWithVoltage(DesiredState desiredState, double desiredVoltage) {
    this.desiredState = desiredState;
    this.desiredVoltage = desiredVoltage;
  }
}
