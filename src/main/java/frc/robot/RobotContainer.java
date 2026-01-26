package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.team6328.LocalADStarAK;
import frc.robot.auto.Auto1Test;
import frc.robot.auto.Auto2Test;
import frc.robot.auto.NoneAuto;
import frc.robot.simulation.SimulatedRobotState;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveIOCTRE;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem.DesiredState;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.rollers.RollerSubsystem;
import frc.robot.subsystems.rollers.RolllerIOTalonFx;
import frc.robot.subsystems.rollers.RolllerIOTalonFx2;
import frc.robot.subsystems.vision.VisionPoseEstimateInField;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;
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
          DRIVE_CONTROLLER,
          TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
          TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
    } else {
      return new DriveSubsystem(
          new DriveIOCTRE(
              robotState,
              DriveConstants.SWERVE_DRIVETRAIN.getDrivetrainConstants(),
              DriveConstants.SWERVE_DRIVETRAIN.getModuleConstants()),
          robotState,
          DRIVE_CONTROLLER,
          TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
          TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
    }
  }

  private RollerSubsystem buildRollerSubsystem() {

    return new RollerSubsystem(new RolllerIOTalonFx());
  }

  private RollerSubsystem buildRollerSubsystem2() {
    return new RollerSubsystem(new RolllerIOTalonFx2());
  }

  // -- Controller
  private final CommandXboxController DRIVE_CONTROLLER = new CommandXboxController(0);

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
  private final RollerSubsystem rollerSub = buildRollerSubsystem();
  private final RollerSubsystem rollerSub2 = buildRollerSubsystem2();

  // -- AutoChooser
  private final LoggedDashboardChooser<frc.robot.auto.AutoBuilder> autoChooser =
      new LoggedDashboardChooser<>("Auto Chooser");
  public static Field2d autoPrev = new Field2d();

  public RobotContainer() {
    if (RobotBase.isSimulation()) {
      assert this.simulatedRobotState != null;
      this.simulatedRobotState.init();
    }

    driveSub.setState(DesiredState.MANUAL_FIELD_DRIVE);
    configureButtonBindings(DRIVE_CONTROLLER);
    configureAuto();
  }

  public void configureButtonBindings(CommandXboxController controller) {
    controller
        .a()
        .whileTrue(
            Commands.run(() -> driveSub.setDesiredPointToLock(new Translation2d(4.626, 4.033))))
        .onFalse(
            Commands.runOnce(
                () -> driveSub.setState(DriveSubsystem.DesiredState.MANUAL_FIELD_DRIVE)));
    
    controller
        .leftTrigger()
        .whileTrue(
            Commands.run(
                () ->
                    rollerSub.setDesiredStateWithVoltage(RollerSubsystem.DesiredState.FORWARD, 7)))
        .onFalse(
            Commands.runOnce(
                () -> rollerSub.setDesiredState(RollerSubsystem.DesiredState.STOPPED)));
    controller
        .rightTrigger()
        .whileTrue(
            Commands.run(
                () ->
                    rollerSub.setDesiredStateWithVoltage(RollerSubsystem.DesiredState.REVERSE, 7)))
        .onFalse(
            Commands.runOnce(
                () -> rollerSub.setDesiredState(RollerSubsystem.DesiredState.STOPPED)));
    // Para los rollers
    controller
        .leftBumper()
        .whileTrue(
            Commands.run(
                () ->
                    rollerSub2.setDesiredStateWithVoltage(RollerSubsystem.DesiredState.FORWARD, 8)))
        .onFalse(
            Commands.runOnce(
                () -> rollerSub2.setDesiredState(RollerSubsystem.DesiredState.STOPPED)));
    controller
        .rightBumper()
        .whileTrue(
            Commands.run(
                () ->
                    rollerSub2.setDesiredStateWithVoltage(RollerSubsystem.DesiredState.REVERSE, 8)))
        .onFalse(
            Commands.runOnce(
                () -> rollerSub2.setDesiredState(RollerSubsystem.DesiredState.STOPPED)));
  }

  public void configureAuto() {
    autoChooser.addDefaultOption("None Auto", new NoneAuto());
    autoChooser.addOption("Testing Auto", new Auto1Test());
    autoChooser.addOption("Testing Auto 2", new Auto2Test());

    autoChooser.onChange(
        auto -> {
          if (auto != null) {
            autoPrev.getObject("path").setPoses(auto.getPathPoses());
          }
        });

    PathPlannerLogging.setLogActivePathCallback(
        (poses -> Logger.recordOutput("Autonomous/ActivePath", poses.toArray(new Pose2d[0]))));
    PathPlannerLogging.setLogTargetPoseCallback(
        pose -> Logger.recordOutput("Autonomous/TargetPose", pose));

    Pathfinding.setPathfinder(new LocalADStarAK());
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

    SmartDashboard.putData("AutoPrev", autoPrev);
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
