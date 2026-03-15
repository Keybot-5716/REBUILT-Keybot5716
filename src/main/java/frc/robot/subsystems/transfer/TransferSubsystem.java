package frc.robot.subsystems.transfer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.LoggedTunableNumber;
import frc.lib.util.DataProcessor;
import org.littletonrobotics.junction.Logger;

public class TransferSubsystem extends SubsystemBase {
  private final TransferIO transferIO;

  private final TransferIOInputsAutoLogged transferInputs = new TransferIOInputsAutoLogged();

  private DesiredState desiredState = DesiredState.STOPPED;
  private TransferState transferState = TransferState.STOPPING;

  private static final LoggedTunableNumber rollerRPS =
      new LoggedTunableNumber("Transfer/Rollers/RollerRPS", 25.0);

  public enum DesiredState {
    STOPPED,
    FORWARD,
    REVERSE
  }

  private enum TransferState {
    STOPPING,
    FORWARDING,
    REVERSING
  }

  public TransferSubsystem(TransferIO transferIO) {
    this.transferIO = transferIO;
    DataProcessor.initDataProcessor(
        () -> {
          synchronized (transferInputs) {
            transferIO.updateInputs(transferInputs);
          }
        },
        transferIO);
  }

  @Override
  public void periodic() {
    synchronized (transferInputs) {
      Logger.processInputs("Transfer/TransferInputs", transferInputs);

      Logger.recordOutput("Transfer/DesiredState", desiredState);
      Logger.recordOutput("Transfer/CurrentState", transferState);
      transferState = setStateTransition();
      applyStates();
    }
  }

  private TransferState setStateTransition() {
    return switch (desiredState) {
      case STOPPED -> TransferState.STOPPING;
      case FORWARD -> TransferState.FORWARDING;
      case REVERSE -> TransferState.REVERSING;
    };
  }

  private void applyStates() {
    switch (transferState) {
      case FORWARDING:
        setVelocity(rollerRPS.get());
        break;

      case REVERSING:
        setVelocity(-rollerRPS.get());
        break;

      case STOPPING:
        stop();
        break;
    }
  }

  private void stop() {
    transferIO.stopMotor();
  }

  private void setVelocity(double rps) {
    transferIO.setVelocity(rps);
  }

  public void setDesiredState(DesiredState desiredState) {
    this.desiredState = desiredState;
  }
}
