package frc.robot.subsystems.vision.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {

  @AutoLog
  public class RollerIOInputs {
    public boolean motorConnected;
    public double acceleration;
    public double appliedVolts;
    public double tempCelcius;
  }

  void updateInputs(RollerIOInputs inputs);

  void setRollerSpeed(double output);

  void stopRoller();

  void runOpenLoop(double output);
}
