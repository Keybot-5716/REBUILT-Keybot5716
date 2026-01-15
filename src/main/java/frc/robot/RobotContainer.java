package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

public class RobotContainer {

  // -- Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // -- AutoChooser
  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());

    configureButtonBindings();
  }

  public void configureButtonBindings() {
    controller.a().onTrue(Commands.none());
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
