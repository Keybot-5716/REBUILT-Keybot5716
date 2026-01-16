package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.team254.ConcurrentTimeInterpolatableBuffer;

public class RobotState {
  public static final double BUFFER_TIME = 1.0;

  public RobotState() {
    fieldToRobot.addSample(0.0, Pose2d.kZero);
  }

  private final ConcurrentTimeInterpolatableBuffer<Pose2d> fieldToRobot =
      ConcurrentTimeInterpolatableBuffer.createBuffer(BUFFER_TIME);
}
