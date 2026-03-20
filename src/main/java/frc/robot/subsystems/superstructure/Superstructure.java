package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.pivot.IntakePivotSubsystem;
import frc.robot.subsystems.intake.rollers.IntakeRollersSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.transfer.TransferSubsystem;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

  private final DriveSubsystem driveSub;
  private final IntakePivotSubsystem intakePivotSubsystem;
  private final IntakeRollersSubsystem intakeRollersSub;
  private final TransferSubsystem transferSub;
  private final ShooterSubsystem shooterSub;
  private final RobotState robotState;

  private SuperstructureStates currentState = SuperstructureStates.DEFAULT;
  private SuperstructureStates desiredState = SuperstructureStates.DEFAULT;

  public Superstructure(
      DriveSubsystem driveSub,
      IntakePivotSubsystem intakePivotSubsystem,
      IntakeRollersSubsystem intakeRollersSub,
      TransferSubsystem transferSub,
      ShooterSubsystem shooterSub,
      RobotState robotState) {
    this.driveSub = driveSub;
    this.intakePivotSubsystem = intakePivotSubsystem;
    this.intakeRollersSub = intakeRollersSub;
    this.transferSub = transferSub;
    this.shooterSub = shooterSub;
    this.robotState = robotState;
  }

  public void setDesiredState(SuperstructureStates targetState) {
    this.desiredState = targetState;
  }

  @Override
  public void periodic() {
    if (currentState != desiredState) {
      currentState = desiredState;
    }
    Logger.recordOutput("Superstructure/DesiredState", desiredState);
    Logger.recordOutput("Superstructure/CurrentState", currentState);

    tryStates();
  }

  private void tryStates() {
    switch (currentState) {
      case DEFAULT:
        def();
        break;

      case HOME:
        home();
        break;

      case INTAKE:
        intake();
        break;

      case SCORE:
        score();
        break;

      case TAXI:
        taxi();
        break;

      case EJECT:
        def();
        break;
    }
  }

  private void def() {
    driveSub.setState(DriveSubsystem.DesiredState.MANUAL_FIELD_DRIVE);
    intakePivotSubsystem.setDesiredState(IntakePivotSubsystem.DesiredState.IN);
    intakeRollersSub.setDesiredState(IntakeRollersSubsystem.DesiredState.STOPPED);
    transferSub.setDesiredState(TransferSubsystem.DesiredState.STOPPED);
    shooterSub.setDesiredState(ShooterSubsystem.DesiredState.STOPPED);
  }

  private void home() {
    driveSub.setState(DriveSubsystem.DesiredState.MANUAL_FIELD_DRIVE);
    intakePivotSubsystem.setDesiredState(IntakePivotSubsystem.DesiredState.IN);
    intakeRollersSub.setDesiredState(IntakeRollersSubsystem.DesiredState.STOPPED);
    transferSub.setDesiredState(TransferSubsystem.DesiredState.STOPPED);
    shooterSub.setDesiredState(ShooterSubsystem.DesiredState.STOPPED);
  }

  private void intake() {
    driveSub.setState(DriveSubsystem.DesiredState.MANUAL_FIELD_DRIVE);
    intakePivotSubsystem.setDesiredState(IntakePivotSubsystem.DesiredState.OUT);
    transferSub.setDesiredState(TransferSubsystem.DesiredState.STOPPED);
    shooterSub.setDesiredState(ShooterSubsystem.DesiredState.STOPPED);

    if (intakePivotSubsystem.isOut()) {
      intakeRollersSub.setDesiredState(IntakeRollersSubsystem.DesiredState.FORWARD_ROLLERS);
    } else {
      intakeRollersSub.setDesiredState(IntakeRollersSubsystem.DesiredState.STOPPED);
    }
  }

  private void score() {

    driveSub.setDesiredPointToLock(FieldConstants.getHubShootingPose().getTranslation());
    intakePivotSubsystem.setDesiredState(IntakePivotSubsystem.DesiredState.IN);
    intakeRollersSub.setDesiredState(IntakeRollersSubsystem.DesiredState.STOPPED);
    shooterSub.setDesiredState(ShooterSubsystem.DesiredState.FORWARD_ROLLERS);

    if (driveSub.isAlignedToPoint()) {
      transferSub.setDesiredState(TransferSubsystem.DesiredState.FORWARD);
    } else {
      transferSub.setDesiredState(TransferSubsystem.DesiredState.STOPPED);
    }
  }

  private void taxi() {
    driveSub.setDesiredRotationToLock(new Rotation2d(robotState.isRedAlliance() ? 0 : Math.PI));
    intakePivotSubsystem.setDesiredState(IntakePivotSubsystem.DesiredState.IN);
    intakeRollersSub.setDesiredState(IntakeRollersSubsystem.DesiredState.STOPPED);
    shooterSub.setDesiredState(ShooterSubsystem.DesiredState.FORWARD_ROLLERS);

    if (driveSub.isAlignedToAngle()) {
      transferSub.setDesiredState(TransferSubsystem.DesiredState.FORWARD);
    } else {
      transferSub.setDesiredState(TransferSubsystem.DesiredState.STOPPED);
    }
  }

  public Command setCommand(Superstructure superstructure, SuperstructureStates states) {
    return new InstantCommand(() -> superstructure.setDesiredState(states));
  }
}
