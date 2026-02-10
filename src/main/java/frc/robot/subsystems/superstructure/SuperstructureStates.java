package frc.robot.subsystems.superstructure;

import java.util.EnumSet;
import java.util.Set;

import frc.robot.subsystems.superstructure.SuperstructureConstants.IntakeConstants;

public enum SuperstructureStates {
    DEFAULT(IntakeConstants.ZERO_RVOLTAGE,0.0,0.0),
    HOME(IntakeConstants.ZERO_RVOLTAGE,0.0,0.0),
    PREP_INTAKE(IntakeConstants.ZERO_RVOLTAGE,0.0,0.0),
    INTAKING(7.0,0.0,0.0),
    PREP_SHOOTING(IntakeConstants.ZERO_RVOLTAGE,0.0,0.0),
    SHOOTING(IntakeConstants.ZERO_RVOLTAGE,4.0,5.0),
    PREP_CLIMBING(IntakeConstants.ZERO_RVOLTAGE,0.0,0.0),
    CLIMBING(IntakeConstants.ZERO_RVOLTAGE,0.0,0.0);

    private final double intakeRollerSpeed;
    private final double transferRollerSpeed;
    private final double shooterRollerSpeed;

    SuperstructureStates(
        double intakeRollerSpeed,
        double transferRollerSpeed,
        double shooterRollerSpeed
    ) {
        this.intakeRollerSpeed = intakeRollerSpeed;
        this.transferRollerSpeed = transferRollerSpeed;
        this.shooterRollerSpeed = shooterRollerSpeed;
    }

    public double getIntakeRollerSpeed() {
        return intakeRollerSpeed;
    }

    public double getTransferRollerSpeed() {
        return transferRollerSpeed;
    }

    public double getShooterRollerSpeed() {
        return shooterRollerSpeed;
    }

    public Set<SuperstructureStates> allowedStates() {
        Set<SuperstructureStates> states = EnumSet.allOf(SuperstructureStates.class);
        switch (this) {
            case DEFAULT:
                states.removeAll(EnumSet.of(CLIMBING));
                return states;
            
            default:
                return states;
        }
    }

    public boolean isIntake() {
        switch (this) {
            case INTAKING:
            case PREP_INTAKE:
                return true;
            default:
                return false;
        }
    }

    public boolean isShooter() {
        switch (this) {
            case PREP_SHOOTING:
            case SHOOTING:
                return true;
            default:
                return false;
        }
    }

    public boolean isClimber() {
        switch (this) {
            case CLIMBING:
            case PREP_CLIMBING:
                return true;
            default:
                return false;
        }
    }
}
