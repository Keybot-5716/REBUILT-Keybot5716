package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class SuperstructureCommands {

  public Command setCommand(Superstructure superstructure, SuperstructureStates states) {
    return Commands.run(() -> superstructure.setDesiredState(states));
  }
}
