package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.vision.rollers.RollerIOSim;
import frc.robot.subsystems.vision.rollers.RollerSubsystem;
import frc.robot.subsystems.vision.rollers.RolllerIOTalonFx;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  // -- Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // -- AutoChooser
  private final LoggedDashboardChooser<Command> autoChooser;

  private RollerSubsystem buildRollerSubsystem() {
    if (RobotBase.isSimulation()) {
      return new RollerSubsystem(new RollerIOSim());
    } else {
      // Cambiar si es que es SparkMax
      return new RollerSubsystem(new RolllerIOTalonFx());
    }
  }

  // SUBSYSTEMS
  RollerSubsystem rollerSub = buildRollerSubsystem();

  public RobotContainer() {
    autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());

    configureButtonBindings();
  }

  public void configureButtonBindings() {
    controller.a().onTrue(Commands.runOnce(() -> rollerSub.setDesiredStateWithSpeed(RollerSubsystem.DesiredState.FORWARD, 0.75)));
    controller.b().onTrue(Commands.runOnce(() -> rollerSub.setDesiredStateWithSpeed(RollerSubsystem.DesiredState.FORWARD, 0.6)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
