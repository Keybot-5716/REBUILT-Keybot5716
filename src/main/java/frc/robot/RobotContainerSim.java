package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.team6328.LocalADStarAK;
import frc.lib.util.RobotCore;
import frc.lib.util.Visualizer;
import frc.robot.auto.Auto1Test;
import frc.robot.auto.Auto2Test;
import frc.robot.auto.Auto3Test;
import frc.robot.auto.Auto4Test;
import frc.robot.auto.AutoBuilder;
import frc.robot.auto.AutoForwardTest;
import frc.robot.auto.NoneAuto;
import frc.robot.simulation.SimulatedRobotState;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.intake.pivot.IntakePivotIOSim;
import frc.robot.subsystems.vision.VisionPoseEstimateInField;
import frc.robot.subsystems.visualizers.RobotSimVisualizer;
import java.util.function.Consumer;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.ironmaple.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainerSim implements RobotCore {

  private DriveSubsystem buildDriveSubsystem() {
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
  }

  private final Consumer<VisionPoseEstimateInField> visionFieldEstimate =
      new Consumer<VisionPoseEstimateInField>() {
        @Override
        public void accept(VisionPoseEstimateInField estimation) {
          driveSub.addVisionMeasurement(estimation);
        }
      };

  private DriveSubsystem driveSub = buildDriveSubsystem();

  private IntakePivotIOSim buildIntakePivotSim() {
    return new IntakePivotIOSim(driveSub.getMapleSimDrive().mapleSimDrive);
  }

  private final LoggedDashboardChooser<AutoBuilder> autoChooser =
      new LoggedDashboardChooser<>("Auto Chooser");

  private IntakePivotIOSim intakePivotSubSim = buildIntakePivotSim();

  private final RobotState robotState = new RobotState(visionFieldEstimate);

  private final CommandXboxController DRIVE_CONTROLLER = new CommandXboxController(0);

  private final RobotSimVisualizer robotVisualizer =
      new RobotSimVisualizer(robotState, intakePivotSubSim);

  public static Field2d autoPrev = new Field2d();

  SimulatedRobotState simulatedRobotState =
      RobotBase.isSimulation() ? new SimulatedRobotState(this) : null;

  public RobotContainerSim() {
    assert this.simulatedRobotState != null;
    this.simulatedRobotState.init();
    configureButtonBindingsSim(DRIVE_CONTROLLER);
  }

  public void configureAuto() {
    autoChooser.addDefaultOption("None Auto", new NoneAuto());
    autoChooser.addOption("Testing Auto", new Auto1Test());
    autoChooser.addOption("Testing Auto 2", new Auto2Test());
    autoChooser.addOption("Auto Forward", new AutoForwardTest());
    autoChooser.addOption("Testing Auto 3", new Auto3Test());
    autoChooser.addOption("Testing Auto 4", new Auto4Test());

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

  public void configureButtonBindingsSim(CommandXboxController controller) {
    controller
        .rightBumper()
        .whileTrue(
            Commands.run(
                () ->
                    driveSub.setDesiredPointToLock(
                        new Translation2d(
                            FieldConstants.getHubShootingPose().getX(),
                            FieldConstants.getHubShootingPose().getY()))))
        .onFalse(
            Commands.runOnce(
                () -> driveSub.setState(DriveSubsystem.DesiredState.MANUAL_FIELD_DRIVE)));
    controller
        .start()
        .onTrue(Commands.runOnce(() -> SimulatedArena.getInstance().resetFieldForAuto()));
    controller.povUp().onTrue(Commands.runOnce(() -> intakePivotSubSim.resetIntake()));
    controller.a().onTrue(Commands.runOnce(() -> intakePivotSubSim.setRunning(true)));
    controller.b().onTrue(Commands.runOnce(() -> intakePivotSubSim.setRunning(false)));

    controller
        .rightTrigger()
        .whileTrue(
            Commands.sequence(
                Commands.either(
                        Commands.run(
                            () ->
                                driveSub.setDesiredPointToLock(
                                    new Translation2d(
                                        FieldConstants.getHubShootingPose().getX(),
                                        FieldConstants.getHubShootingPose().getY()))),
                        Commands.run(
                            () ->
                                driveSub.setDesiredRotationToLock(
                                    new Rotation2d(robotState.isRedAlliance() ? 0 : Math.PI))),
                        () -> !robotState.passedTrench())
                    .withTimeout(0.7),
                Commands.repeatingSequence(
                    Commands.either(
                        Commands.runOnce(() -> generateFuel()),
                        Commands.runOnce(() -> generateFuelTaxi()),
                        () -> !robotState.passedTrench()),
                    Commands.waitSeconds(0.2))))
        .onFalse(
            Commands.runOnce(
                () -> driveSub.setState(DriveSubsystem.DesiredState.MANUAL_FIELD_DRIVE)));
  }

  public void generateFuel() {
    IntakeSimulation intakeSimulation = intakePivotSubSim.getIntakeSimulation();
    if (intakePivotSubSim.isFuelInsideIntake()) {
      RebuiltFuelOnFly fuelOnFly =
          new RebuiltFuelOnFly(
              // Specify the position of the chassis when the note is launched
              driveSub.getDesiredPoint(),
              // Specify the translation of the shooter from the robot center (in the shooter’s
              // reference frame)
              new Translation2d(-0.2, -0.2),
              // Specify the field-relative speed of the chassis, adding it to the initial velocity
              // of
              // the projectile
              robotState.getLatestFusedFieldRelativeChassisSpeeds(),
              // The shooter facing direction is the same as the robot’s facing direction
              robotState
                  .getLatestFieldToRobot()
                  .getValue()
                  .getRotation()
                  .minus(new Rotation2d(Math.PI / 2)),
              // Add the shooter’s rotation
              // + shooterRotation,
              // Initial height of the flying note
              Distance.ofRelativeUnits(0.56, Meters),
              // The launch speed is proportional to the RPM
              // RPM
              LinearVelocity.ofRelativeUnits(7, MetersPerSecond),
              // The angle at which the fuel is launched
              Angle.ofRelativeUnits(54, Degrees));

      fuelOnFly
          // Set the target center to the Rebbuilt Hub of the current alliance
          .withTargetPosition(
              () ->
                  FieldMirroringUtils.toCurrentAllianceTranslation(new Translation3d(4.6, 4, 2.3)))
          // Set the tolerance: x: ±0.5m, y: ±1.2m, z: ±0.3m (this is the size of the speaker's
          // "mouth")
          .withTargetTolerance(new Translation3d(0.5, 1.2, 0.3))
          // Set a callback to run when the fuel hits the target
          .withHitTargetCallBack(() -> System.out.println("Hit hub, +1 point!"));

      fuelOnFly
          // Configure callbacks to visualize the flight trajectory of the projectile
          .withProjectileTrajectoryDisplayCallBack(
          // Callback for when the fuel will eventually hit the target (if configured)
          (pose3ds) ->
              Logger.recordOutput(
                  "Flywheel/FuelProjectileSuccessfulShot", pose3ds.toArray(Pose3d[]::new)),
          // Callback for when the fuel will eventually miss the target, or if no target is
          // configured
          (pose3ds) ->
              Logger.recordOutput(
                  "Flywheel/FuelProjectileUnsuccessfulShot", pose3ds.toArray(Pose3d[]::new)));

      // Add the projectile to the simulated arena
      SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);

      intakeSimulation.setGamePiecesCount(intakePivotSubSim.getFuelAmount() - 1);
    }
  }

  private void generateFuelTaxi() {
    IntakeSimulation intakeSimulation = intakePivotSubSim.getIntakeSimulation();
    if (intakePivotSubSim.isFuelInsideIntake()) {
      RebuiltFuelOnFly fuelOnFly =
          new RebuiltFuelOnFly(
              // Specify the position of the chassis when the note is launched
              driveSub.getDesiredPoint(),
              // Specify the translation of the shooter from the robot center (in the shooter’s
              // reference frame)
              new Translation2d(0.2, 0),
              // Specify the field-relative speed of the chassis, adding it to the initial velocity
              // of
              // the projectile
              robotState.getLatestFusedFieldRelativeChassisSpeeds(),
              // The shooter facing direction is the same as the robot’s facing direction
              robotState.getLatestFieldToRobot().getValue().getRotation(),
              // Add the shooter’s rotation
              // + shooterRotation,
              // Initial height of the flying note
              Distance.ofRelativeUnits(1, Meters),
              // The launch speed is proportional to the RPM; assumed to be 16 meters/second at 6000
              // RPM
              LinearVelocity.ofRelativeUnits(7, MetersPerSecond),
              // The angle at which the fuel is launched
              Angle.ofRelativeUnits(3, Degrees));

      fuelOnFly
          // Set the target center to the Rebbuilt Hub of the current alliance
          .withTargetPosition(
              () ->
                  FieldMirroringUtils.toCurrentAllianceTranslation(new Translation3d(4.6, 4, 2.3)))
          // Set the tolerance: x: ±0.5m, y: ±1.2m, z: ±0.3m (this is the size of the speaker's
          // "mouth")
          .withTargetTolerance(new Translation3d(0.5, 1.2, 0.3))
          // Set a callback to run when the fuel hits the target
          .withHitTargetCallBack(() -> System.out.println("Hit hub, +1 point!"));

      fuelOnFly
          // Configure callbacks to visualize the flight trajectory of the projectile
          .withProjectileTrajectoryDisplayCallBack(
          // Callback for when the fuel will eventually hit the target (if configured)
          (pose3ds) ->
              Logger.recordOutput(
                  "Flywheel/FuelProjectileSuccessfulShot", pose3ds.toArray(Pose3d[]::new)),
          // Callback for when the fuel will eventually miss the target, or if no target is
          // configured
          (pose3ds) ->
              Logger.recordOutput(
                  "Flywheel/FuelProjectileUnsuccessfulShot", pose3ds.toArray(Pose3d[]::new)));

      // Add the projectile to the simulated arena
      SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);

      intakeSimulation.setGamePiecesCount(intakePivotSubSim.getFuelAmount() - 1);
    }
  }

  public RobotState getRobotState() {
    return robotState;
  }

  public IntakePivotIOSim getIntakePivotIOSim() {
    return intakePivotSubSim;
  }

  public DriveSubsystem getDriveSubsystem() {
    return driveSub;
  }

  public SimulatedRobotState getSimRobotState() {
    return simulatedRobotState;
  }

  public Visualizer getRobotVisualizer() {
    return robotVisualizer;
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
