package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

public class DriveConstants {

  public static final double ROBOT_MASS = 50;
  public static final double BUMPER_LENGTH_INCHES = 30;
  public static final double BUMPER_WIDTH_INCHES = 30;

  public static final double DRIVE_TO_POSE_STATIC_FRICTION_CONSTANT = 0.02;

  public static final PIDController teleopAutoAllignController = new PIDController(3.0, 0, 0.1);
  public static final PIDController autoAutoAllignController = new PIDController(3.0, 0, 0.1);

  public static final double maxVelocityToAutoAllign = Units.feetToMeters(8.0);

  public static final CommandSwerveDrivetrain SWERVE_DRIVETRAIN = TunerConstants.createDrivetrain();

  public static final double MAX_SPEED = 4.58;
  public static final double WHEEL_COF = 1.0;
}
