package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.team6328.LocalADStarAK;
import frc.lib.util.RobotCore;
import frc.robot.auto.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.intake.pivot.*;
import frc.robot.subsystems.intake.rollers.*;
import frc.robot.subsystems.shooter.ShootCalculator;
import frc.robot.subsystems.shooter.hood.*;
import frc.robot.subsystems.shooter.rollers.*;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureStates;
import frc.robot.subsystems.transfer.*;
import frc.robot.subsystems.vision.VisionIOHardwareLimelight;
import frc.robot.subsystems.vision.VisionPoseEstimateInField;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.visualizers.RobotVisualizer;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer implements RobotCore {
  private DriveSubsystem buildDriveSubsystem() {
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

  //  -- Intake
  private IntakePivotSubsystem buildIntakePivot() {
    return new IntakePivotSubsystem(new IntakePivotIOTalonFX(), robotState);
  }

  private IntakeRollersSubsystem buildIntakeRollers() {
    return new IntakeRollersSubsystem(new IntakeRollerIOTalonFX());
  }

  // -- Shooter
  private ShooterHoodSubsystem buildShooterHood() {
    return new ShooterHoodSubsystem(new ShooterHoodIOTalonFX());
  }

  private ShooterRollersSubsystem buildShooterRollers() {
    return new ShooterRollersSubsystem(new ShooterRollersIOTalonFX());
  }

  // -- Transfer
  private TransferSubsystem buildTransfer() {
    return new TransferSubsystem(new TrasnferIOTalonFX());
  }

  private Superstructure buildSuperstructure() {
    return new Superstructure(
        driveSub,
        intakePivotSub,
        intakeRollersSub,
        transferSub,
        shooterHoodSub,
        shooterRollersSub,
        shootCalculator,
        robotState);
  }

  private VisionSubsystem buildVisionSubsystem() {
    return new VisionSubsystem(new VisionIOHardwareLimelight(robotState), robotState);
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

  // -- Subsystems
  private final DriveSubsystem driveSub = buildDriveSubsystem();
  private final ShooterHoodSubsystem shooterHoodSub = buildShooterHood();
  private final ShooterRollersSubsystem shooterRollersSub = buildShooterRollers();
  private final TransferSubsystem transferSub = buildTransfer();
  private final IntakeRollersSubsystem intakeRollersSub = buildIntakeRollers();
  private final IntakePivotSubsystem intakePivotSub = buildIntakePivot();
  private final VisionSubsystem visionSub = buildVisionSubsystem();
  private final ShootCalculator shootCalculator = new ShootCalculator(robotState);

  private final Superstructure superstructure = buildSuperstructure();

  // -- AutoChooser
  private final LoggedDashboardChooser<AutoBuilder> autoChooser =
      new LoggedDashboardChooser<>("Auto Chooser");
  public static Field2d autoPrev = new Field2d();

  private final RobotVisualizer robotVisualizer = new RobotVisualizer(robotState);

  public RobotContainer() {
    configureButtonBindings(DRIVE_CONTROLLER);
    NamedCommands.registerCommand(
        "SCORE",
        Commands.run(
            () -> {
              driveSub.setDesiredPointToLock(FieldConstants.getHubShootingPose().getTranslation());
              intakeRollersSub.setDesiredState(IntakeRollersSubsystem.DesiredState.STOPPED);
              shooterRollersSub.setCustom(50.5);
              shooterHoodSub.setDesiredState(ShooterHoodSubsystem.DesiredState.CALC_POS_TO_SCORE);

              if (driveSub.isAlignedToPoint() && shooterRollersSub.atDesiredVelocity()) {
                transferSub.setDesiredState(TransferSubsystem.DesiredState.OSCILLATE_FORWARD);
              }
            }));
    configureAuto();
    driveSub.setState(DriveSubsystem.DesiredState.MANUAL_FIELD_DRIVE);
  }

  /**
   * Este metodo configura los botones
   *
   * @param controller es un mando de xbox
   */
  public void configureButtonBindings(CommandXboxController controller) {
    controller
        .start()
        .onTrue(
            Commands.runOnce(() -> driveSub.resetOdometry(FieldConstants.getRightTrenchTesting())));
    controller
        .back()
        .onTrue(Commands.runOnce(() -> driveSub.resetOdometry(FieldConstants.getTaxiPose())));
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
        .onTrue(superstructure.setCommand(SuperstructureStates.TAXI, ShootCalculator.hubPreset))
        .onFalse(superstructure.setCommand(SuperstructureStates.HOME));

    controller
        .leftTrigger()
        .onTrue(superstructure.setCommand(SuperstructureStates.INTAKE))
        .onFalse(superstructure.setCommand(SuperstructureStates.DEFAULT));

    controller
        .a()
        .whileTrue(
            Commands.run(
                () -> transferSub.setDesiredState(TransferSubsystem.DesiredState.OSCILLATE_FORWARD),
                transferSub))
        .onFalse(
            Commands.runOnce(
                () -> transferSub.setDesiredState(TransferSubsystem.DesiredState.STOPPED),
                transferSub));

    controller
        .b()
        .onTrue(superstructure.setPresetCommand(ShootCalculator.hubPreset))
        .onFalse(superstructure.setCommand(SuperstructureStates.HOME));

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

    // controller.povLeft().onTrue(superstructureCommands.setCommand(superstructure,
    // SuperstructureStates.HOME));
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
    autoChooser.addOption("Right Trench", new AutoRightTrench());
    autoChooser.addOption("Right Outpost", new AutoRightOutpost());
    autoChooser.addOption("MechTest", new AutoMech());

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
    /*
    NamedCommands.registerCommand("SCORE", superstructure.setCommand(SuperstructureStates.SCORE));
    NamedCommands.registerCommand("HOME", superstructure.setCommand(SuperstructureStates.HOME));
    NamedCommands.registerCommand("INTAKE", superstructure.setCommand(SuperstructureStates.INTAKE));
    NamedCommands.registerCommand("TAXI", superstructure.setCommand(SuperstructureStates.TAXI));
    NamedCommands.registerCommand("DEFAULT", superstructure.setCommand(SuperstructureStates.DEFAULT));*/
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

  public RobotVisualizer getRobotVisualizer() {
    return robotVisualizer;
  }

  public ShootCalculator getShootCalculator() {
    return shootCalculator;
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
