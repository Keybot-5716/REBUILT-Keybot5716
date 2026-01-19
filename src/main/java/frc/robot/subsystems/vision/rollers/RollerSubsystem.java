package frc.robot.subsystems.vision.rollers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerSubsystem extends SubsystemBase {
  // Creamos un objeto de nuestra interfaz para manejar todo desde ahí
  private final RollerIO io;

  // Con los enums del estado deseado y el estado actual para poder modificarlos y saber en qué
  // punto están al simularlos
  private DesiredState desiredState = DesiredState.STOPPED;
  private RollersState rollersState = RollersState.STOPPING;

  private double desiredSpeed;

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

  public RollerSubsystem(RollerIO io /*RobotState robotState*/) {
    this.io = io;
    // this.robotState = robotState;
  }

  @Override
  public void periodic() {
    // Hacer sistema para loggeo
  }

  public void stop() {
    io.stopRoller();
  }

  public void setRollerSpeed(double speed){
    io.setRollerSpeed(speed);
  }

  private RollersState setStateTransition(){
    return switch (desiredState){
      case STOPPED -> RollersState.STOPPING;
      case FORWARD -> RollersState.FORWARDING;
      case REVERSE -> RollersState.REVERSING;
    };
  }

  private void applyStates(){
    switch (rollersState) {
      case FORWARDING:
        setRollerSpeed(desiredSpeed);
        break;
      
      case REVERSING:
        setRollerSpeed(-desiredSpeed);

      case STOPPING:
        setRollerSpeed(0);
    }
  }

  public void setDesiredState(DesiredState desiredState) {
    this.desiredState = desiredState;
  }

  public void setDesiredStateWithSpeed(DesiredState desiredState, double desiredSpeed) {
    this.desiredState = desiredState;
    this.desiredSpeed = desiredSpeed;
  }
}
