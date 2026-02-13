package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.shooter.hood.ShooterHoodIO;
import frc.robot.subsystems.shooter.rollers.ShooterRollersIO;
import frc.robot.subsystems.shooter.rollers.ShooterRollersIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  private final ShooterRollersIO rollersIO;
  private final ShooterHoodIO hoodIO;

  // private final ShooterRollerIOInputsAutoLogged inputs = new ShooterRollerIOInputsAutoLogged();
  private final ShooterRollersIOInputsAutoLogged rollersInputs =
      new ShooterRollersIOInputsAutoLogged();

  private static final LoggedTunableNumber rollerVolts =
      new LoggedTunableNumber("Intake/Rollers/RollerVolts", 7.0);

  // Creamos un objeto TrapezoidProfile que tenga velocidad y aceleración máximas
  private TrapezoidProfile.Constraints profile = new TrapezoidProfile.Constraints(2.0, 2.0);

  // Creamos un controller para el trapezoidProfile
  private ProfiledPIDController controller = new ProfiledPIDController(5, 0, 0, profile);

  // Cambiar hasta qué posición queremos que llegue el intake
  private static TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);

  private DesiredState desiredState = DesiredState.STOPPED;
  private ShooterState shooterState = ShooterState.STOPPING;

  private double desiredVoltage;

  public enum DesiredState {
    STOPPED,
    FORWARD_ROLLERS,
    REVERSE_ROLLERS,
    IN,
    OUT,
    TEST
  }

  private enum ShooterState {
    STOPPING,
    FORWARDING_ROLLERS,
    REVERSING_ROLLERS,
    INING,
    OUTING,
    TESTING
  }

  public ShooterSubsystem(ShooterRollersIO rollersIO, ShooterHoodIO hoodIO) {
    this.rollersIO = rollersIO;
    this.hoodIO = hoodIO;
  }

  @Override
  public void periodic() {
    /*????
    if (DriverStation.isDisabled()) {
      controller.reset(inputs.positionIntake);
    }
      */

    Logger.processInputs("ShooterInputs/RollerInputs", rollersInputs);
    rollersIO.updateInputs(rollersInputs);

    shooterState = setStateTransition();
    applyStates();
  }

  private ShooterState setStateTransition() {
    return switch (desiredState) {
      case STOPPED -> ShooterState.STOPPING;
      case FORWARD_ROLLERS -> ShooterState.FORWARDING_ROLLERS;
      case REVERSE_ROLLERS -> ShooterState.REVERSING_ROLLERS;
      case IN -> ShooterState.INING;
      case OUT -> ShooterState.OUTING;
      case TEST -> ShooterState.TESTING;
    };
  }

  public void stop() {
    rollersIO.stopRollers();
  }

  public void setVoltageRollers(double voltage) {
    rollersIO.setVoltage(voltage);
  }

  public void setVoltageIntakeTesters(double voltage) {
    // io.setVoltage(voltage);
  }

  private void applyStates() {
    switch (shooterState) {
      case FORWARDING_ROLLERS:
        setVoltageRollers(rollerVolts.get());
        break;

      case REVERSING_ROLLERS:
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
