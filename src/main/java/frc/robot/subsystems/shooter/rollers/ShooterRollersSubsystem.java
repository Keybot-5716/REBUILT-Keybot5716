package frc.robot.subsystems.shooter.rollers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.LoggedTunableNumber;
import frc.lib.util.DataProcessor;
import org.littletonrobotics.junction.Logger;

public class ShooterRollersSubsystem extends SubsystemBase {
  private final ShooterRollersIO rollersIO;

  private final ShooterRollersIOInputsAutoLogged rollersInputs =
      new ShooterRollersIOInputsAutoLogged();

  private DesiredState desiredState = DesiredState.STOPPED;
  private ShooterRollersState rollerState = ShooterRollersState.STOPPING;

  private static final LoggedTunableNumber desiredVelocityRollers =
      new LoggedTunableNumber("Shooter/Rollers/RollerVelocity", 56.7);

  public enum DesiredState {
    STOPPED,
    FORWARD_ROLLERS,
    REVERSE_ROLLERS,
  }

  private enum ShooterRollersState {
    STOPPING,
    FORWARDING_ROLLERS,
    REVERSING_ROLLERS,
  }

  public ShooterRollersSubsystem(ShooterRollersIO rollersIO) {
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
      Logger.processInputs("Shooter/RollerInputs", rollersInputs);

      Logger.recordOutput("Shooter/DesiredState", desiredState);
      Logger.recordOutput("Shooter/ShooterSollerState", rollerState);

      rollerState = setStateTransition();
      applyStates();
    }
  }

  private ShooterRollersState setStateTransition() {
    return switch (desiredState) {
      case STOPPED -> ShooterRollersState.STOPPING;
      case FORWARD_ROLLERS -> ShooterRollersState.FORWARDING_ROLLERS;
      case REVERSE_ROLLERS -> ShooterRollersState.REVERSING_ROLLERS;
    };
  }

  private void applyStates() {
    switch (rollerState) {
      case FORWARDING_ROLLERS:
        setVelocityRollers(desiredVelocityRollers.get());
        break;

      case REVERSING_ROLLERS:
        setVelocityRollers(-desiredVelocityRollers.get());
        break;

      case STOPPING:
        stop();
        break;
    }
  }

  private void stop() {
    rollersIO.stopMotor();
  }

  public void setDesiredState(DesiredState state) {
    this.desiredState = state;
  }

  public void setVelocityRollers(double rps) {
    rollersIO.setVelocity(rps);
  }
}
