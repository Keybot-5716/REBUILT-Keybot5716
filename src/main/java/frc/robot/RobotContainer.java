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
import frc.robot.auto.*;
import frc.robot.simulation.SimulatedRobotState;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.intake.pivot.*;
import frc.robot.subsystems.intake.rollers.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.shooter.hood.*;
import frc.robot.subsystems.shooter.rollers.*;
import frc.robot.subsystems.transfer.*;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOSimPhotonVision;
import frc.robot.subsystems.vision.VisionPoseEstimateInField;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.visualizers.RobotVisualizer;
import java.util.function.Consumer;
import org.ironmaple.simulation.IntakeSimulation;
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
  // private RollerSubsystem buildRollerSubsystem() {return new RollerSubsystem(new
  // RolllerIOTalonFx());}

  //  -- Intake
  private IntakePivotSubsystem buildIntakePivot() {
    return new IntakePivotSubsystem(new IntakePivotIOTalonFX(), robotState);
  }

  private IntakePivotIOSim buildIntakePivotSim() {
    return new IntakePivotIOSim(driveSub.getMapleSimDrive().mapleSimDrive);
  }

  private IntakeRollersSubsystem buildIntakeRollers() {
    return new IntakeRollersSubsystem(new IntakeRollerIOTalonFX());
  }

  // -- Shooter
  private ShooterSubsystem buildShooter() {
    return new ShooterSubsystem(new ShooterRollersIOTalonFX(), new ShooterHoodIOTalonFX());
  }

  // -- Transfer
  private TransferSubsystem buildTransfer() {
    return new TransferSubsystem(new TrasnferIOTalonFX());
  }

  private VisionSubsystem buildVisionSubsystem() {
    if (Robot.isSimulation()) {
      return new VisionSubsystem(
          new VisionIOSimPhotonVision(robotState, simulatedRobotState), robotState);
    } else {
      return new VisionSubsystem(new VisionIOLimelight(robotState), robotState);
    }
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
  private final ShooterSubsystem shooterSub = buildShooter();
  private final TransferSubsystem transferSub = buildTransfer();
  private final IntakeRollersSubsystem intakeRollersSub = buildIntakeRollers();
  private final IntakePivotSubsystem intakePivotSub = buildIntakePivot();
  private final IntakePivotIOSim intakePivotSubSim = buildIntakePivotSim();

  private final VisionSubsystem visionSub = buildVisionSubsystem();
  // private final IntakePivotIOSim intakePivotSub = new
  // IntakePivotIOSim(driveSub.getMapleSimDrive().mapleSimDrive);

  // -- AutoChooser
  private final LoggedDashboardChooser<AutoBuilder> autoChooser =
      new LoggedDashboardChooser<>("Auto Chooser");
  public static Field2d autoPrev = new Field2d();

  private final RobotVisualizer robotVisualizer = new RobotVisualizer(robotState);

  public RobotContainer() {
    if (RobotBase.isSimulation()) {
      assert this.simulatedRobotState != null;
      this.simulatedRobotState.init();
      configureButtonBindingsSim(DRIVE_CONTROLLER);
    } else {
      configureButtonBindings(DRIVE_CONTROLLER);
    }
    configureAuto();
    driveSub.setState(DriveSubsystem.DesiredState.MANUAL_FIELD_DRIVE);
  }

  /**
   * Este metodo configura los botones
   *
   * @param controller es un mando de xbox
   */
  public void configureButtonBindings(CommandXboxController controller) {
    controller.start().onTrue(Commands.runOnce(() -> driveSub.resetOdometry()));
    controller
        .start()
        .onTrue(Commands.runOnce(() -> driveSub.resetOdometry(FieldConstants.getTestingPose())));
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
        .rightTrigger()
        .whileTrue(
            Commands.run(
                () -> shooterSub.setDesiredState(ShooterSubsystem.DesiredState.FORWARD_ROLLERS)))
        .onFalse(
            Commands.runOnce(
                () -> shooterSub.setDesiredState(ShooterSubsystem.DesiredState.STOPPED)));
    controller
        .b()
        .whileTrue(
            Commands.run(() -> transferSub.setDesiredState(TransferSubsystem.DesiredState.FORWARD)))
        .onFalse(
            Commands.runOnce(
                () -> transferSub.setDesiredState(TransferSubsystem.DesiredState.STOPPED)));
    controller
        .a()
        .whileTrue(
            Commands.run(() -> transferSub.setDesiredState(TransferSubsystem.DesiredState.REVERSE)))
        .onFalse(
            Commands.runOnce(
                () -> transferSub.setDesiredState(TransferSubsystem.DesiredState.STOPPED)));

    controller
        .x()
        .whileTrue(
            Commands.run(
                () ->
                    intakeRollersSub.setDesiredState(
                        IntakeRollersSubsystem.DesiredState.FORWARD_ROLLERS)))
        .onFalse(
            Commands.runOnce(
                () ->
                    intakeRollersSub.setDesiredState(IntakeRollersSubsystem.DesiredState.STOPPED)));

    controller
        .y()
        .whileTrue(
            Commands.run(
                () ->
                    intakeRollersSub.setDesiredState(
                        IntakeRollersSubsystem.DesiredState.REVERSE_ROLLERS)))
        .onFalse(
            Commands.runOnce(
                () ->
                    intakeRollersSub.setDesiredState(IntakeRollersSubsystem.DesiredState.STOPPED)));

    controller
        .povUp()
        .onTrue(
            Commands.runOnce(
                () -> intakePivotSub.setDesiredState(IntakePivotSubsystem.DesiredState.IN)));
    controller
        .povDown()
        .onTrue(
            Commands.runOnce(
                () -> intakePivotSub.setDesiredState(IntakePivotSubsystem.DesiredState.OUT)));
  }

  /**
   * Este metodo configura los botones para el simulador
   *
   * @param controller es un mando de xbox
   *     <p>public void configureButtonBindingsSim(CommandXboxController controller) {
   *     controller.a().onTrue(Commands.runOnce(() -> generateFuel()));
   *     controller.b().onTrue(Commands.runOnce(() -> intakePivotSub.setRunning(true)));
   *     controller.x().onTrue(Commands.runOnce(() -> intakePivotSub.setRunning(false))); controller
   *     .start() .onTrue(Commands.runOnce(() -> SimulatedArena.getInstance().resetFieldForAuto()));
   *     <p>controller .rightTrigger() .whileTrue( Commands.sequence( Commands.either( Commands.run(
   *     () -> driveSub.setDesiredPointToLock(new Translation2d(4.626, 4.033))), Commands.run( () ->
   *     driveSub.setDesiredRotationToLock( new Rotation2d(robotState.isRedAlliance() ? 0 :
   *     Math.PI))), () -> !robotState.passedTrench()) .withTimeout(0.7),
   *     Commands.repeatingSequence( Commands.either( Commands.runOnce(() -> generateFuel()),
   *     Commands.runOnce(() -> generateFuelTaxi()), () -> !robotState.passedTrench()),
   *     Commands.waitSeconds(0.2)))) .onFalse( Commands.runOnce( () ->
   *     driveSub.setState(DriveSubsystem.DesiredState.MANUAL_FIELD_DRIVE))); }
   *     <p>/** Este metodo genera los fuels dependiento de la posición del chasis, la velocidad
   *     incicial del fuel, la dirección en la cual se va a estar lanzando la velocidad y el ángulo
   *     al que será lanzado
   */
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
        .x()
        .whileTrue(Commands.run(() -> intakePivotSubSim.launchFuel()))
        .onFalse(Commands.run(() -> intakePivotSubSim.stopMotor()));

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

  /*
   * Configura las diferentes rutas que se pueden utilizar para el manejo en el auto
   */
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

  public DriveSubsystem getDriveSubsystem() {
    return driveSub;
  }

  public VisionSubsystem getVisionSubsystem() {
    return visionSub;
  }

  public RobotState getRobotState() {
    return robotState;
  }

  public SimulatedRobotState getSimRobotState() {
    return simulatedRobotState;
  }

  public RobotVisualizer getRobotVisualizer() {
    return robotVisualizer;
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public IntakePivotIOSim getIntakePivotIOSim() {
    return intakePivotSubSim;
  }
}
