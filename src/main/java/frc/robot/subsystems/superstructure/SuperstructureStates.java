package frc.robot.subsystems.superstructure;

import java.util.EnumSet;
import java.util.Set;

public enum SuperstructureStates {
  DEFAULT,
  HOME,
  INTAKE,
  SCORE,
  TAXI,
  EJECT;

  public boolean isScoring() {
    return this == SCORE;
  }

  public boolean isIntaking() {
    return this == INTAKE;
  }

  public Set<SuperstructureStates> allowedNextStates() {
    switch (this) {
      case DEFAULT:
        return EnumSet.of(INTAKE);

      case INTAKE:
        return EnumSet.of(HOME, EJECT);

      case HOME:
        return EnumSet.of(INTAKE, SCORE, EJECT);

      case SCORE:
        return EnumSet.of(HOME);

      case TAXI:
        return EnumSet.of(HOME);

      default:
        return EnumSet.allOf(SuperstructureStates.class);
    }
  }
}
