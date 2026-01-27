package frc.robot.subsystems.angle;

import org.littletonrobotics.junction.AutoLog;

public interface angleIO {
  @AutoLog
  public static class angleIOInputs {
    angleIOData data = new angleIOData(false, 0, 0, 0, 0, 0);
  }

  record angleIOData(
      boolean motorConnected,
      double positionRotations,
      double velocityRotationsPerSec,
      double appliedVolts,
      double supplyCurrentAmps,
      double tempCelsius) {}

  public default void updateInputs(angleIOInputs inputs) {}

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
