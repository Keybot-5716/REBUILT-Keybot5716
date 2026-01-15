package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

public class CommandSwerveDrivetrain {
  SwerveDrivetrainConstants drivetrainConstants;
  SwerveModuleConstants<?, ?, ?>[] moduleConstants;

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    this.drivetrainConstants = drivetrainConstants;
    this.moduleConstants = modules;
  }

  public SwerveDrivetrainConstants getDrivetrainConstants() {
    return drivetrainConstants;
  }

  public SwerveModuleConstants<?, ?, ?>[] getModuleConstants() {
    return moduleConstants;
  }
}
