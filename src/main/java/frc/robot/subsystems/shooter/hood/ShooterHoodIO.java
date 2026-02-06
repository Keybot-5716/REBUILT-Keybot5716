package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterHoodIO {

  @AutoLog
  public static class HoodIOInputs {

    HoodIOData data = new HoodIOData(false, 0, 0, 0, 0, 0);
  }

  record HoodIOData(
      boolean motorConnected,
      double positionRotations,
      double velocityRotationsPerSec,
      double appliedVolts,
      double supplyCurrentAmps,
      double tempCelsius) {}

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void stop() {}

  public default void runOpenLoop(double output) {}

  public default void setVoltage(double volts) {}

  public default void setPosition(double position) {}

  public default void setNeutralModeBreak(boolean enable) {}

  public default void resetEncoder() {}

  public default void setPID(double kP, double kI, double kD) {}

  /** Se utiliza para protocolos SysID */
  default void optimizeForSysID() {}
}
