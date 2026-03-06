package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.shooter.hood.HoodIOInputsAutoLogged;
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

  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

  private static final LoggedTunableNumber rollerVolts =
      new LoggedTunableNumber("Shooter/Rollers/RollerVolts", 1.0);

  // Creamos un objeto TrapezoidProfile que tenga velocidad y aceleración máximas
  private TrapezoidProfile.Constraints profile = new TrapezoidProfile.Constraints(2.0, 2.0);

  // Creamos un controller para el trapezoidProfile
  private ProfiledPIDController controller = new ProfiledPIDController(5, 0, 0, profile);

  // Cambiar hasta qué posición queremos que llegue el intake
  private static TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);

  private DesiredState desiredState = DesiredState.STOPPED;
  private ShooterState shooterState = ShooterState.STOPPING;

  private double desiredVoltage = 0;
  private double desiredVelocity = 0;

  private static final LoggedTunableNumber desiredVelocityTunable =
      new LoggedTunableNumber("Shooter/Rollers/RollerVelocity", 50.0);

  public enum DesiredState {
    STOPPED,
    FORWARD_ROLLERS,
    REVERSE_ROLLERS,
    IN,
    OUT,
    TEST,
    SCORE,
    TAXI
  }

  private enum ShooterState {
    STOPPING,
    FORWARDING_ROLLERS,
    REVERSING_ROLLERS,
    INING,
    OUTING,
    TESTING,
    SCORING,
    TAXIING
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

    Logger.processInputs("RollerInputs", rollersInputs);
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
      case SCORE -> ShooterState.SCORING;
      case TAXI -> ShooterState.TAXIING;
    };
  }

  public void stop() {
    rollersIO.stopRollers();
  }

  public void stopHood() {
    hoodIO.stop();
  }

  public void setVoltageRollers(double voltage) {
    rollersIO.setVoltage(voltage);
  }

  public void setVelocityRollers(double rps) {
    rollersIO.setVelocity(rps);
  }

  private void applyStates() {
    switch (shooterState) {
      case FORWARDING_ROLLERS:
        setVoltageRollers(desiredVoltage);
        break;

      case REVERSING_ROLLERS:
        setVoltageRollers(-desiredVoltage);
        break;

      case STOPPING:
        setVoltageRollers(0);
        // setPosition(0.0);
        break;

      case INING:
        setPosition(ShooterConstants.In);
        break;

      case OUTING:
        setPosition(ShooterConstants.Out);
        break;

      case TESTING:
        setVelocityRollers(desiredVelocityTunable.get());
        // setPosition(ShooterConstants.hoodScore);
        break;

      case SCORING:
        setPosition(ShooterConstants.hoodScore);
        break;

      case TAXIING:
        setPosition(ShooterConstants.hoodTaxi);
        break;
    }
  }

  public void setPosition(double position) {
    goal.position = position;
    hoodIO.setPosition(position);
    // io.setVoltage(controller.calculate(inputs.positionIntake, goal.position));
  }

  public void setDesiredState(DesiredState desiredState) {
    this.desiredState = desiredState;
  }

  public void setDesiredStateWithVoltage(DesiredState desiredState, double desiredVoltage) {
    this.desiredState = desiredState;
    this.desiredVoltage = desiredVoltage;
  }

  public void setDesiredStateWithVelocity(DesiredState desiredState, double rps) {
    this.desiredState = desiredState;
    this.desiredVelocity = rps;
  }
}
