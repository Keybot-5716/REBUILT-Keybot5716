package frc.robot.subsystems.superstructure;

import frc.robot.subsystems.superstructure.SuperstructureConstants.IntakeConstants;

public enum SuperstructureStates {
  DEFAULT(IntakeConstants.ZERO_RVOLTAGE, 0.0),
  HOME(IntakeConstants.ZERO_RVOLTAGE, 0.0),
  PREP_INTAKE(IntakeConstants.ZERO_RVOLTAGE, 0.0),
  INTAKING(7.0, 0.0),
  PREP_SHOOTING(IntakeConstants.ZERO_RVOLTAGE, 0.0),
  SHOOTING(IntakeConstants.ZERO_RVOLTAGE, 5.0);

  private final double intakeRollerSpeed;
  private final double shooterRollerVelocity;

  SuperstructureStates(double intakeRollerVelocity, double shooterRollerVelocity) {
    this.intakeRollerSpeed = intakeRollerVelocity;
    this.shooterRollerVelocity = shooterRollerVelocity;
  }

  public double getIntakeRollerSpeed() {
    return intakeRollerSpeed;
  }

  public double getShooterRollerSpeed() {
    return shooterRollerVelocity;
  }

  public boolean isIntake() {
    return this == INTAKING || this == PREP_INTAKE;
  }

  public boolean isShooter() {
    return this == PREP_SHOOTING || this == SHOOTING;
  }
}
