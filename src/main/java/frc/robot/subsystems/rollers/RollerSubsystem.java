package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerSubsystem extends SubsystemBase {
  // Creamos un objeto de nuestra interfaz para manejar todo desde ahí
  private final RollerIO io;
  private RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

  // Con los enums del estado deseado y el estado actual para poder modificarlos y saber en qué
  // punto están al simularlos
  private DesiredState desiredState = DesiredState.STOPPED;
  private RollersState rollersState = RollersState.STOPPING;

  private double desiredVoltage;

  // RobotState robotState;

  public enum DesiredState {
    STOPPED,
    FORWARD,
    REVERSE
  }

  private enum RollersState {
    STOPPING,
    FORWARDING,
    REVERSING
  }

  public RollerSubsystem(RollerIO io /*, RobotState robotState*/) {
    this.io = io;
    // this.robotState = robotState;
  }

  @Override
  public void periodic() {
    // Hacer sistema para loggeo
    io.updateInputs(inputs);
    Logger.recordOutput("Rollers/ CurrentVoltage", inputs.appliedVolts);
    // Para que los estados funcionen
    rollersState = setStateTransition();
    applyStates();
  }

  public void stop() {
    io.stopRoller();
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  private RollersState setStateTransition() {
    return switch (desiredState) {
      case STOPPED -> RollersState.STOPPING;
      case FORWARD -> RollersState.FORWARDING;
      case REVERSE -> RollersState.REVERSING;
    };
  }

  private void applyStates() {
    switch (rollersState) {
      case FORWARDING:
        setVoltage(desiredVoltage);
        break;

      case REVERSING:
        setVoltage(-desiredVoltage);

      case STOPPING:
        setVoltage(0);
    }
  }

  public void setDesiredState(DesiredState desiredState) {
    this.desiredState = desiredState;
  }

  public void setDesiredStateWithVoltage(DesiredState desiredState, double desiredVoltage) {
    this.desiredState = desiredState;
    this.desiredVoltage = desiredVoltage;
  }
}
