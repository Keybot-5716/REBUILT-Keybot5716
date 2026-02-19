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
import frc.robot.auto.Auto1Test;
import frc.robot.auto.Auto2Test;
import frc.robot.auto.AutoBuilder;
import frc.robot.auto.AutoForwardTest;
import frc.robot.auto.NoneAuto;
import frc.robot.simulation.SimulatedRobotState;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveIOCTRE;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem.DesiredState;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.pivot.IntakePivotIOSim;
import frc.robot.subsystems.intake.pivot.IntakePivotIOTalonFX;
import frc.robot.subsystems.intake.rollers.IntakeRollersIOSparkMax;
import frc.robot.subsystems.rollers.RollerSparkMax;
import frc.robot.subsystems.rollers.RollerSubsystem;
import frc.robot.subsystems.rollers.RolllerIOTalonFx;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.shooter.hood.*;
import frc.robot.subsystems.shooter.rollers.*;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionPoseEstimateInField;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.function.Consumer;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.ironmaple.utils.FieldMirroringUtils;
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

  // -- Rollers genéricos
  private RollerSubsystem buildRollerSubsystem() {
    return new RollerSubsystem(new RolllerIOTalonFx());
  }

  // -- Intake
  private IntakeSubsystem buildIntake() {
    if (RobotBase.isSimulation()) {
      return new IntakeSubsystem(
          new IntakePivotIOSim(driveSub.getMapleSimDrive().mapleSimDrive),
          new IntakeRollersIOSparkMax());
    } else {
      return new IntakeSubsystem(new IntakePivotIOTalonFX(), new IntakeRollersIOSparkMax());
    }
  }

  // -- Shooter
  private ShooterSubsystem buildShooter() {
    return new ShooterSubsystem(new ShooterRollersIOTalonFX(), new ShooterHoodIOTalonFX());
  }

  private RollerSubsystem buildTransfer() {
    return new RollerSubsystem(new RollerSparkMax(21));
  }

  private VisionSubsystem buildVisionSubsystem() {
    return new VisionSubsystem(new VisionIOLimelight(robotState), robotState);
  }

  private final Consumer<VisionPoseEstimateInField> visionFieldEstimate =
      new Consumer<VisionPoseEstimateInField>() {
        @Override
        public void accept(VisionPoseEstimateInField estimation) {
          driveSub.addVisionMeasurement(estimation);
        }
      };

  /*private final Consumer<VisionPoseEstimateInField> visionFieldEstimate =
  new Consumer<VisionPoseEstimateInField>() {
    @Override
    public void accept(VisionPoseEstimateInField estimation) {

      if (driveSub == null) return;
      if (estimation == null || estimation.getRobotPose() == null) return;
      Pose2d p = estimation.getRobotPose();
      if (Double.isNaN(p.getX()) || Double.isNaN(p.getY())) return;
      driveSub.addVisionMeasurement(estimation);
    }
  }; */

  private final CommandXboxController DRIVE_CONTROLLER = new CommandXboxController(0);

  private final RobotState robotState = new RobotState(visionFieldEstimate);

  private final SimulatedRobotState simulatedRobotState =
      RobotBase.isSimulation() ? new SimulatedRobotState(this) : null;

  // -- Subsystems
  private final DriveSubsystem driveSub = buildDriveSubsystem();
  private final RollerSubsystem rollerSub = buildRollerSubsystem();
  // private final RollerSubsystem intakeRollerSub = buildIntakeRoller();
  private final ShooterSubsystem shooterSub = buildShooter();
  private final IntakeSubsystem intakeSub = buildIntake();
  private final RollerSubsystem transferRoller = buildTransfer();
  // private final IntakeSubsystem intakePivotSub = buildIntakePivotSubsystem();
  private final VisionSubsystem visionSub = buildVisionSubsystem();
  private final IntakePivotIOSim intakePivotSub =
      new IntakePivotIOSim(driveSub.getMapleSimDrive().mapleSimDrive);

  // -- AutoChooser
  private final LoggedDashboardChooser<AutoBuilder> autoChooser =
      new LoggedDashboardChooser<>("Auto Chooser");
  public static Field2d autoPrev = new Field2d();

  public RobotContainer() {
    if (RobotBase.isSimulation()) {
      assert this.simulatedRobotState != null;
      this.simulatedRobotState.init();
      configureButtonBindingsSim(DRIVE_CONTROLLER);
    } else {
      configureButtonBindings(DRIVE_CONTROLLER);
    }
    configureAuto();
    driveSub.setState(DesiredState.MANUAL_FIELD_DRIVE);
  }

  public void configureButtonBindings(CommandXboxController controller) {
    // Para el shooter (delante y pa' tras xd)
    controller
        .rightTrigger()
        .whileTrue(
            Commands.run(
                () -> shooterSub.setDesiredState(ShooterSubsystem.DesiredState.FORWARD_ROLLERS)))
        .whileFalse(
            Commands.run(() -> shooterSub.setDesiredState(ShooterSubsystem.DesiredState.STOPPED)));

    controller
        .rightBumper()
        .whileTrue(
            Commands.run(
                () -> shooterSub.setDesiredState(ShooterSubsystem.DesiredState.REVERSE_ROLLERS)))
        .whileFalse(
            Commands.run(() -> shooterSub.setDesiredState(ShooterSubsystem.DesiredState.STOPPED)));

    // Para el intake

    controller
        .rightBumper()
        .whileTrue(
            Commands.run(
                () -> intakeSub.setDesiredState(IntakeSubsystem.DesiredState.FORWARD_ROLLERS)))
        .whileFalse(
            Commands.run(
                () -> intakeSub.setDesiredState(IntakeSubsystem.DesiredState.REVERSE_ROLLERS)));

    controller
        .leftBumper()
        .whileTrue(
            Commands.run(
                () -> intakeSub.setDesiredState(IntakeSubsystem.DesiredState.REVERSE_ROLLERS)))
        .whileFalse(
            Commands.run(
                () -> intakeSub.setDesiredState(IntakeSubsystem.DesiredState.REVERSE_ROLLERS)));

    // Para probar el pivot del intake Con a de arriba y b de bajar
    controller
        .a()
        .onTrue(
            Commands.run(
                () -> intakeSub.setDesiredState(IntakeSubsystem.DesiredState.FORWARD_PIVOT)));
    controller
        .b()
        .onTrue(
            Commands.run(
                () -> intakeSub.setDesiredState(IntakeSubsystem.DesiredState.REVERSE_PIVOT)));

    // Para probar el pivot del intake con posiciones predefinidas (x de arriba y y de abajo)
    controller
        .x()
        .onTrue(Commands.run(() -> intakeSub.setDesiredState(IntakeSubsystem.DesiredState.IN)));
    controller
        .y()
        .onTrue(Commands.run(() -> intakeSub.setDesiredState(IntakeSubsystem.DesiredState.OUT)));

    // Para el shooter
    /*
     * controller
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
                    intakeSub.setDesiredStateWithVoltage(IntakeSubsystem.DesiredState.FORWARD, 8)))
        .onFalse(
            Commands.runOnce(
                () -> intakeSub.setDesiredState(IntakeSubsystem.DesiredState.STOPPED)));
    controller
        .rightBumper()
        .whileTrue(
            Commands.run(
                () ->
                    intakeSub.setDesiredStateWithVoltage(IntakeSubsystem.DesiredState.REVERSE, 8)))
        .onFalse(
            Commands.runOnce(
                () -> intakeSub.setDesiredState(IntakeSubsystem.DesiredState.STOPPED)));

    // Para el intake pivot
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                () -> intakePivotSub.setDesiredState(IntakeSubsystem.DesiredState.IN)));
    controller
        .x()
        .onTrue(
            Commands.runOnce(
                () -> intakePivotSub.setDesiredState(IntakeSubsystem.DesiredState.OUT)));

     */
    controller
        .start()
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
    controller
        .leftBumper()
        .whileTrue(
            Commands.run(
                () ->
                    intakeSub.setDesiredStateWithVoltage(
                        IntakeSubsystem.DesiredState.FORWARD_PIVOT, 1)))
        .onFalse(
            Commands.runOnce(
                () -> intakeSub.setDesiredState(IntakeSubsystem.DesiredState.STOPPED)));
    controller
        .rightBumper()
        .whileTrue(
            Commands.run(
                () ->
                    intakeSub.setDesiredStateWithVoltage(
                        IntakeSubsystem.DesiredState.REVERSE_PIVOT, 1)))
        .onFalse(
            Commands.runOnce(
                () -> intakeSub.setDesiredState(IntakeSubsystem.DesiredState.STOPPED)));

    controller
        .a()
        .whileTrue(
            Commands.run(
                () ->
                    transferRoller.setDesiredStateWithVoltage(
                        RollerSubsystem.DesiredState.FORWARD, 10)))
        .onFalse(
            Commands.runOnce(
                () -> transferRoller.setDesiredState(RollerSubsystem.DesiredState.STOPPED)));

    controller
        .b()
        .whileTrue(
            Commands.run(
                () ->
                    transferRoller.setDesiredStateWithVoltage(
                        RollerSubsystem.DesiredState.REVERSE, 10)))
        .onFalse(
            Commands.runOnce(
                () -> transferRoller.setDesiredState(RollerSubsystem.DesiredState.STOPPED)));
  }

  public void configureButtonBindingsSim(CommandXboxController controller) {

    controller.a().onTrue(Commands.runOnce(() -> generateFuel()));
    controller.b().onTrue(Commands.runOnce(() -> intakePivotSub.setRunning(true)));
    controller.x().onTrue(Commands.runOnce(() -> intakePivotSub.setRunning(false)));
    controller
        .start()
        .onTrue(Commands.runOnce(() -> SimulatedArena.getInstance().resetFieldForAuto()));

    controller
        .rightTrigger()
        .whileTrue(
            Commands.sequence(
                Commands.either(
                        Commands.run(
                            () -> driveSub.setDesiredPointToLock(new Translation2d(4.626, 4.033))),
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

  private void generateFuel() {
    RebuiltFuelOnFly fuelOnFly =
        new RebuiltFuelOnFly(
            // Specify the position of the chassis when the note is launched
            driveSub.getDesiredPoint(),
            // Specify the translation of the shooter from the robot center (in the shooter’s
            // reference frame)
            new Translation2d(0.2, 0),
            // Specify the field-relative speed of the chassis, adding it to the initial velocity of
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
            Angle.ofRelativeUnits(36, Degrees));

    fuelOnFly
        // Set the target center to the Rebbuilt Hub of the current alliance
        .withTargetPosition(
            () -> FieldMirroringUtils.toCurrentAllianceTranslation(new Translation3d(4.6, 4, 2.3)))
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
        // Callback for when the fuel will eventually miss the target, or if no target is configured
        (pose3ds) ->
            Logger.recordOutput(
                "Flywheel/FuelProjectileUnsuccessfulShot", pose3ds.toArray(Pose3d[]::new)));

    // Add the projectile to the simulated arena
    SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);
  }

  private void generateFuelTaxi() {
    RebuiltFuelOnFly fuelOnFly =
        new RebuiltFuelOnFly(
            // Specify the position of the chassis when the note is launched
            driveSub.getDesiredPoint(),
            // Specify the translation of the shooter from the robot center (in the shooter’s
            // reference frame)
            new Translation2d(0.2, 0),
            // Specify the field-relative speed of the chassis, adding it to the initial velocity of
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
            () -> FieldMirroringUtils.toCurrentAllianceTranslation(new Translation3d(4.6, 4, 2.3)))
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
        // Callback for when the fuel will eventually miss the target, or if no target is configured
        (pose3ds) ->
            Logger.recordOutput(
                "Flywheel/FuelProjectileUnsuccessfulShot", pose3ds.toArray(Pose3d[]::new)));

    // Add the projectile to the simulated arena
    SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);
  }

  public void configureAuto() {
    autoChooser.addDefaultOption("None Auto", new NoneAuto());
    autoChooser.addOption("Testing Auto", new Auto1Test());
    autoChooser.addOption("Testing Auto 2", new Auto2Test());
    autoChooser.addOption("Auto Forward", new AutoForwardTest());c

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

  // public VisionSubsystem getVisionSubsystem() {
  // return visionSub;
  // }

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
