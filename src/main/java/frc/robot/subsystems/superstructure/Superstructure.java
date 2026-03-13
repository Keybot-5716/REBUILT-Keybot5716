package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.rollers.IntakeRollersSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.transfer.TransferSubsystem;

public class Superstructure extends SubsystemBase {

  private final IntakeRollersSubsystem intakeSub;
  private final TransferSubsystem transferSub;
  private final ShooterSubsystem shooterSub;

  private SuperstructureStates currentState = SuperstructureStates.DEFAULT;
  private SuperstructureStates desiredState = SuperstructureStates.DEFAULT;

  public Superstructure(
      IntakeRollersSubsystem intakeSub,
      TransferSubsystem transferSub,
      ShooterSubsystem shooterSub) {
    this.intakeSub = intakeSub;
    this.transferSub = transferSub;
    this.shooterSub = shooterSub;
  }

  @Override
  public void periodic() {
    currentState = desiredState;
  }
}
