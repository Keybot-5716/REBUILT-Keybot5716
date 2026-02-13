package frc.robot.subsystems.transfer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class TransferSubsystem extends SubsystemBase {
  TransferIO transferIO;

  private final TransferIOInputsAutoLogged transferInputs = new TransferIOInputsAutoLogged();

  private DesiredState desiredState = DesiredState.STOPPED;
  private TransferState transferState = TransferState.STOPPING;

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
  }

  @Override
  public void periodic() {
    Logger.processInputs("TransferSubsystem", transferInputs);
    transferIO.updateInputs(transferInputs);
  }

  public void stop() {
    transferIO.stopRollers();
  }
}
