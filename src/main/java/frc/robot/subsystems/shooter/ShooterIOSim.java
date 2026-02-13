package frc.robot.subsystems.shooter;

import frc.robot.subsystems.shooter.rollers.ShooterRollersIO;

public class ShooterIOSim implements ShooterRollersIO {

  public void launchFuel() {}

  @Override
  public void updateInputs(ShooterRollersIOInputs inputs) {}

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public void stopRollers() {}
}
