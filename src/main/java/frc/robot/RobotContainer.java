package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveIOCTRE;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem.DesiredState;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.vision.VisionPoseEstimateInField;
import java.util.function.Consumer;

public class RobotContainer {

  private DriveSubsystem buildDriveSubsystem() {
    return new DriveSubsystem(
        new DriveIOCTRE(
            robotState,
            DriveConstants.SWERVE_DRIVETRAIN.getDrivetrainConstants(),
            DriveConstants.SWERVE_DRIVETRAIN.getModuleConstants()),
        robotState,
        controller,
        TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond),
        TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond));
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

  // -- Subsystems
  private final DriveSubsystem driveSub = buildDriveSubsystem();

  // -- AutoChooser
  // private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    // autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());

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

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
