package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.simulation.SimulatedRobotState;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveIOCTRE;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem.DesiredState;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.vision.VisionPoseEstimateInField;
import java.util.function.Consumer;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  private DriveSubsystem buildDriveSubsystem() {
    if (RobotBase.isSimulation()) {
      return new DriveSubsystem(
          new DriveIOSim(
              robotState,
              simulatedRobotState,
              DriveConstants.SWERVE_DRIVETRAIN.getDrivetrainConstants(),
              DriveConstants.SWERVE_DRIVETRAIN.getModuleConstants()),
          robotState,
          controller,
          TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
          TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
    } else {
      return new DriveSubsystem(
          new DriveIOCTRE(
              robotState,
              DriveConstants.SWERVE_DRIVETRAIN.getDrivetrainConstants(),
              DriveConstants.SWERVE_DRIVETRAIN.getModuleConstants()),
          robotState,
          controller,
          TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
          TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
    }
  }

  // -- Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  private final Consumer<VisionPoseEstimateInField> visionFieldEstimate =
      new Consumer<VisionPoseEstimateInField>() {
        @Override
        public void accept(VisionPoseEstimateInField estimation) {
          driveSub.addVisionMeasurement(estimation);
        }
      };

  private final RobotState robotState = new RobotState(visionFieldEstimate);

  private final SimulatedRobotState simulatedRobotState =
      RobotBase.isSimulation() ? new SimulatedRobotState(this) : null;
  // -- Subsystems
  private final DriveSubsystem driveSub = buildDriveSubsystem();

  // -- AutoChooser
  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());
    autoChooser.addOption("Example Auto", new PathPlannerAuto("New Auto"));

    if (RobotBase.isSimulation()) {
      assert this.simulatedRobotState != null;
      this.simulatedRobotState.init();
    }

    driveSub.setState(DesiredState.MANUAL_FIELD_DRIVE);
    configureButtonBindings(controller);
  }

  public void configureButtonBindings(CommandXboxController control) {
    control.a().onTrue(Commands.none());
  }

  public DriveSubsystem getDriveSubsystem() {
    return driveSub;
  }

  public RobotState getRobotState() {
    return robotState;
  }

  public SimulatedRobotState getSimRobotState() {
    return simulatedRobotState;
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
