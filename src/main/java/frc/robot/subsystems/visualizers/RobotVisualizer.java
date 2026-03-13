package frc.robot.subsystems.visualizers;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class RobotVisualizer {
  private RobotState state;

  private Pose3d intakePose = new Pose3d();

  public RobotVisualizer(RobotState state) {
    this.state = state;
  }

  public void updateRobotVisualizer() {
    /*
    double intakeWidth = 0.5; // Example width of the intake
    double intakeHeight = 0.3; // Example height of the intake
    */
    intakePose = new Pose3d(0, 0, 0, new Rotation3d());

    Logger.recordOutput("ComponentsPoseArray", new Pose3d[] {intakePose});
  }
}
