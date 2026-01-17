package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.VisionPoseEstimateInField;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveSubsystem extends SubsystemBase {

  DriveIO io;
  DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
  DriveTelemetry telemetry = new DriveTelemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

  RobotState robotState;

  CommandXboxController controller;

  Controller pathplannerController;

  private final double maxVelocity;
  private final double maxAngularVelocity;

  private double teleopVelocityCoefficient = 1.0;
  private double rotationVelocityCoefficient = 1.0;

  private Rotation2d desiredRotationToLock = new Rotation2d();
  private Pose2d desiredPoseToAutoAllign = new Pose2d();

  private final ApplyRobotSpeeds stopRequest =
      new ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final ApplyRobotSpeeds pathplannerRequest =
      new ApplyRobotSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withDesaturateWheelSpeeds(true);

  private final SwerveRequest.FieldCentricFacingAngle rotationLocked =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
  private final SwerveRequest.FieldCentricFacingAngle autoAllign =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

  public enum DesiredState {
    STOPPED,
    MANUAL_FIELD_DRIVE,
    ROTATION_LOCK,
    DRIVE_TO_POSE
  }

  private enum SubsystemState {
    STOPPED,
    MANUAL_FIELD_DRIVE,
    ROTATED_TO_ANGLE,
    DRIVEN_TO_POSE
  }

  private DesiredState desiredState = DesiredState.MANUAL_FIELD_DRIVE;
  private SubsystemState currentState = SubsystemState.MANUAL_FIELD_DRIVE;

  public DriveSubsystem(
      DriveIO io,
      RobotState robotState,
      CommandXboxController controller,
      double maxVelocity,
      double maxAngularVelocity) {
    this.io = io;
    this.robotState = robotState;
    this.controller = controller;
    this.maxVelocity = maxVelocity;
    this.maxAngularVelocity = maxAngularVelocity;

    rotationLocked.HeadingController = new PhoenixPIDController(5, 0, 0);
    rotationLocked.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    autoAllign.HeadingController = new PhoenixPIDController(5, 0, 0);
    autoAllign.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  private class Controller implements Consumer<PathPlannerTrajectory>, Runnable {
    private final PathFollowingController PFContoller;
    private volatile PathPlannerTrajectory trajectory = null;
    private volatile Timer Timer = null;
    private Notifier notifier;

    boolean hasPrior = false;

    public Controller(PathFollowingController PFController) {
      this.PFContoller = PFController;
      this.Timer = new Timer();
      notifier = new Notifier(this);
      notifier.startPeriodic(0.01);
    }

    @Override
    public void accept(PathPlannerTrajectory tra) {
      trajectory = tra;
      Timer.reset();
      Timer.start();
      PFContoller.reset(null, null);
    }

    @Override
    public void run() {
      if (!hasPrior) {
        hasPrior = Threads.setCurrentThreadPriority(true, 41);
      }

      PathPlannerTrajectory traj = trajectory;
      if (traj == null) return;
      /**
       * if(traj.isStayStoppedTrajectory()) { setControl(stopRequest); trajectory = null; return; }
       */
      double currentTime = Timer.get();
      PathPlannerTrajectoryState targetState = traj.sample(currentTime);
      ChassisSpeeds speeds =
          PFContoller.calculateRobotRelativeSpeeds(
              robotState.getLatestFieldToRobot().getValue(), targetState);

      if (DriverStation.isEnabled()) {
        setControl(
            pathplannerRequest
                .withSpeeds(speeds)
                .withWheelForceFeedforwardsX(targetState.feedforwards.robotRelativeForcesXNewtons())
                .withWheelForceFeedforwardsY(
                    targetState.feedforwards.robotRelativeForcesYNewtons()));
      }
    }
  }

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();
    io.updateInputs(inputs);
    telemetry.telemeterize(inputs);
    Logger.processInputs("SwerveDriveInputs", inputs);
    io.getModuleStates(inputs);

    robotState.incrementIteration();
    if (DriverStation.isDisabled()) {
      configureStandardDeviationsForDisabled();
    } else {
      configureStandardDeviationsForEnabled();
    }
    Logger.recordOutput("Drive/latencyPeriodicSec", Timer.getFPGATimestamp() - timestamp);
    Logger.recordOutput(
        "Drive/currentCommand",
        (getCurrentCommand() == null) ? "Default" : getCurrentCommand().getName());

    currentState = handleStateTransitions();
    Logger.recordOutput("Drive/States/Current State", currentState);
    Logger.recordOutput("Drive/States/Desired State", desiredState);
    applyStates();
  }

  private void configurePathPlanner() {
    ModuleConfig moduleConfig =
        new ModuleConfig(
            DriveConstants.SWERVE_DRIVETRAIN.getModuleConstants()[0].WheelRadius,
            DriveConstants.MAX_SPEED,
            DriveConstants.WHEEL_COF,
            DCMotor.getKrakenX60(1),
            DriveConstants.SWERVE_DRIVETRAIN.getModuleConstants()[0].DriveMotorGearRatio,
            DriveConstants.SWERVE_DRIVETRAIN.getModuleConstants()[0].SlipCurrent,
            1);

    RobotConfig robotConfig =
        new RobotConfig(
            Constants.ROBOT_MASS_KG,
            Constants.ROBOT_MOI,
            moduleConfig,
            new Translation2d(),
            new Translation2d(),
            new Translation2d(),
            new Translation2d());

    pathplannerController =
        new Controller(
            new PPHolonomicDriveController(
                new PIDConstants(maxAngularVelocity), new PIDConstants(maxAngularVelocity), 0.01));
  }

  private SubsystemState handleStateTransitions() {
    return switch (desiredState) {
      case MANUAL_FIELD_DRIVE -> SubsystemState.MANUAL_FIELD_DRIVE;
      case ROTATION_LOCK -> SubsystemState.ROTATED_TO_ANGLE;
      case DRIVE_TO_POSE -> SubsystemState.DRIVEN_TO_POSE;
      case STOPPED -> SubsystemState.STOPPED;
      default -> SubsystemState.STOPPED;
    };
  }

  private void applyStates() {
    switch (currentState) {
      default:
        break;
      case MANUAL_FIELD_DRIVE:
        manualFieldDrive();
        break;
      case ROTATED_TO_ANGLE:
        rotatedToAngle();
        break;
      case DRIVEN_TO_POSE:
        drivenToPose();
        break;
    }
  }

  public void setState(DesiredState state) {
    this.desiredState = state;
  }

  public void setDesiredRotationToLock(Rotation2d desiredRotation) {
    setState(DesiredState.ROTATION_LOCK);
    this.desiredRotationToLock = desiredRotation;
  }

  public void setDesiredPoseToAutoAllign(Pose2d desiredPose) {
    setState(DesiredState.DRIVE_TO_POSE);
    this.desiredPoseToAutoAllign = desiredPose;
  }

  public void resetOdometry(Pose2d pose) {
    io.resetOdometry(pose);
  }

  public void resetOdometry() {
    io.resetOdometry(new Pose2d());
  }

  public void setControl(SwerveRequest request) {
    io.setRequest(request);
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return io.applyRequest(requestSupplier, this).withName("Swerve drive request");
  }

  public void addVisionMeasurement(VisionPoseEstimateInField fieldEstimation) {
    io.addVisionMeasurement(fieldEstimation);
  }

  public void setStateStdDevs(double x, double y, double r) {
    io.setStateStandardDeviations(x, y, r);
  }

  public void configureStandardDeviationsForDisabled() {
    setStateStdDevs(1, 1, 1);
  }

  public void configureStandardDeviationsForEnabled() {
    setStateStdDevs(0.3, 0.3, 0.2);
  }

  private void manualFieldDrive() {
    var speeds = calculateSpeedsBasedOnJoystickInputs();
    io.setRequest(
        new SwerveRequest.ApplyFieldSpeeds()
            .withSpeeds(speeds)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage));
  }

  private void rotatedToAngle() {
    io.setRequest(
        rotationLocked
            .withVelocityX(calculateSpeedsBasedOnJoystickInputs().vxMetersPerSecond)
            .withVelocityY(calculateSpeedsBasedOnJoystickInputs().vyMetersPerSecond)
            .withTargetDirection(desiredRotationToLock));
  }

  private void drivenToPose() {
    var desiredTranslation =
        desiredPoseToAutoAllign.getTranslation().minus(inputs.Pose.getTranslation());
    var linearDistance = desiredTranslation.getNorm();
    var frictionConstant = 0.0;
    if (linearDistance >= edu.wpi.first.math.util.Units.inchesToMeters(0.5)) {
      frictionConstant = DriveConstants.DRIVE_TO_POSE_STATIC_FRICTION_CONSTANT * maxVelocity;
    }
    var desiredAngle = desiredTranslation.getAngle();
    var velocityOutput = 0.0;
    if (DriverStation.isAutonomous()) {
      velocityOutput =
          Math.min(
              Math.abs(DriveConstants.autoAutoAllignController.calculate(linearDistance, 0))
                  + frictionConstant,
              DriveConstants.maxVelocityToAutoAllign);
    } else {
      velocityOutput =
          Math.min(
              Math.abs(DriveConstants.teleopAutoAllignController.calculate(linearDistance, 0))
                  + frictionConstant,
              DriveConstants.maxVelocityToAutoAllign);
    }

    var xVelocity = velocityOutput * desiredAngle.getCos();
    var yVelocity = velocityOutput * desiredAngle.getSin();

    Logger.recordOutput("Drive/DriveToPose/xVelocity", xVelocity);
    Logger.recordOutput("Drive/DriveToPose/yVelocity", yVelocity);
    Logger.recordOutput("Drive/DriveToPose/velocityOutput", velocityOutput);
    Logger.recordOutput("Drive/DriveToPose/linearDistance", linearDistance);

    io.setRequest(
        autoAllign
            .withVelocityX(xVelocity)
            .withVelocityY(yVelocity)
            .withTargetDirection(desiredPoseToAutoAllign.getRotation()));
  }

  private ChassisSpeeds calculateSpeedsBasedOnJoystickInputs() {
    if (DriverStation.getAlliance().isEmpty()) {
      return new ChassisSpeeds(0, 0, 0);
    }

    double xMagnitude = MathUtil.applyDeadband(controller.getLeftY(), 0.1);
    double yMagnitude = MathUtil.applyDeadband(controller.getLeftX(), 0.1);
    double angularMagnitude = MathUtil.applyDeadband(controller.getRightX(), 0.1);

    angularMagnitude = Math.copySign(angularMagnitude * angularMagnitude, angularMagnitude);

    teleopVelocityCoefficient = controller.leftBumper().getAsBoolean() ? 0.5 : 1.0;

    double xVelocity = -xMagnitude * maxVelocity * teleopVelocityCoefficient;
    double yVelocity = -yMagnitude * maxVelocity * teleopVelocityCoefficient;
    double angularVelocity = angularMagnitude * maxAngularVelocity * rotationVelocityCoefficient;

    Rotation2d skewCompensationFactor =
        Rotation2d.fromRadians(inputs.Speeds.omegaRadiansPerSecond * -0.03);

    return ChassisSpeeds.fromRobotRelativeSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(xVelocity, yVelocity, -angularVelocity), inputs.Pose.getRotation()),
        inputs.Pose.getRotation().plus(skewCompensationFactor));
  }
}
