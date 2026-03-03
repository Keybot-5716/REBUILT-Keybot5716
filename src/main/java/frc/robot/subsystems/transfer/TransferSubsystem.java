package frc.robot.subsystems.transfer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class TransferSubsystem extends SubsystemBase {
  TransferIO transferIO;

  private final TransferIOInputsAutoLogged transferInputs = new TransferIOInputsAutoLogged();

  private DesiredState desiredState = DesiredState.STOPPED;
  private TransferState transferState = TransferState.STOPPING;

  private static final LoggedTunableNumber rollerVolts =
      new LoggedTunableNumber("Transfer/Rollers/RollerVolts", 7.0);

  private double desiredRollersVoltage;

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
        setVoltage(rollerVolts.get());
        break;

      case REVERSING:
        setVoltage(-rollerVolts.get());
        break;

      case STOPPING:
        stop();
        break;
    }
  }

  @Override
  public void periodic() {
    Logger.processInputs("TransferInputs/TransferSubsystem", transferInputs);

    transferIO.updateInputs(transferInputs);

    transferState = setStateTransition();
    applyStates();
  }

  public void stop() {
    transferIO.stopRollers();
  }

  public void setVoltage(double voltage) {
    transferIO.setVoltage(voltage);
  }

  public void setDesiredState(DesiredState desiredState) {
    this.desiredState = desiredState;
  }

  public void setDesiredStateWithVoltage(DesiredState desiredState, double desiredVoltage) {
    this.desiredState = desiredState;
    this.desiredRollersVoltage = desiredVoltage;
  }
}
