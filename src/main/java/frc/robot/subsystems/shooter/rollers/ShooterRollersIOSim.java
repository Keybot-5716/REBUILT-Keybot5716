package frc.robot.subsystems.shooter.rollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class ShooterRollersIOSim implements ShooterRollersIO {

  private RobotContainer container;

  public Command shoot() {
    return Commands.runOnce(() -> container.generateFuel());
  }

  @Override
  public void updateInputs(ShooterRollersIOInputs inputs) {}

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public void stopMotor() {}

  @Override
  public void setVelocity(double rps) {}
}
