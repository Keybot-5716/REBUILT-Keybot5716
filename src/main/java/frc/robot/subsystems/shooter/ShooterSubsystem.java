package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.LoggedTunableNumber;
import frc.lib.util.DataProcessor;
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

  private DesiredState desiredState = DesiredState.STOPPED;
  private ShooterState shooterState = ShooterState.STOPPING;

  private static final LoggedTunableNumber desiredVelocityTunable =
      new LoggedTunableNumber("Shooter/Rollers/RollerVelocity", 56.7);

  public enum DesiredState {
    STOPPED,
    FORWARD_ROLLERS,
    REVERSE_ROLLERS,
    IN,
    OUT,
    SCORE,
    TAXI
  }

  private enum ShooterState {
    STOPPING,
    FORWARDING_ROLLERS,
    REVERSING_ROLLERS,
    INING,
    OUTING,
    SCORING,
    TAXIING
  }

  public ShooterSubsystem(ShooterRollersIO rollersIO, ShooterHoodIO hoodIO) {
    this.rollersIO = rollersIO;
    this.hoodIO = hoodIO;

    DataProcessor.initDataProcessor(
        () -> {
          synchronized (rollersInputs) {
            synchronized (hoodInputs) {
              rollersIO.updateInputs(rollersInputs);
              hoodIO.updateInputs(hoodInputs);
            }
          }
        },
        rollersIO,
        hoodIO);
  }

  @Override
  public void periodic() {
    synchronized (rollersInputs) {
      synchronized (hoodInputs) {
        Logger.processInputs("Shooter/RollerInputs", rollersInputs);
        Logger.processInputs("Shooter/HoodInputs", hoodInputs);

        Logger.recordOutput("Shooter/DesiredState", desiredState);
        Logger.recordOutput("Shooter/CurrentState", shooterState);
        shooterState = setStateTransition();
        applyStates();
      }
    }
  }

  private ShooterState setStateTransition() {
    return switch (desiredState) {
      case STOPPED -> ShooterState.STOPPING;
      case FORWARD_ROLLERS -> ShooterState.FORWARDING_ROLLERS;
      case REVERSE_ROLLERS -> ShooterState.REVERSING_ROLLERS;
      case IN -> ShooterState.INING;
      case OUT -> ShooterState.OUTING;
      case SCORE -> ShooterState.SCORING;
      case TAXI -> ShooterState.TAXIING;
    };
  }

  private void applyStates() {
    switch (shooterState) {
      case FORWARDING_ROLLERS:
        setVelocityRollers(desiredVelocityTunable.get());

        break;

      case REVERSING_ROLLERS:
        setVelocityRollers(-desiredVelocityTunable.get());
        break;

      case STOPPING:
        setVoltageRollers(0);
        // setPosition(0.0);
        break;

      case INING:
        setPosition(0.0);
        break;

      case OUTING:
        setPosition(0.87);
        break;

      case SCORING:
        setPosition(0.3);
        break;

      case TAXIING:
        setPosition(0.9);
        break;
    }
  }

  public void stop() {
    rollersIO.stopMotor();
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

  public void setPosition(double position) {
    hoodIO.setPosition(position);
  }

  public void setDesiredState(DesiredState desiredState) {
    this.desiredState = desiredState;
  }
}
