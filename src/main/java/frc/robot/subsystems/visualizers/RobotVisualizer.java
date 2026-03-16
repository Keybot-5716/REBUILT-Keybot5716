package frc.robot.subsystems.visualizers;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class RobotVisualizer {
  @SuppressWarnings("unused")
  private RobotState state;

  private Pose3d intakePose3d = new Pose3d();

  public RobotVisualizer(RobotState state) {
    this.state = state;
  }

  public void updateRobotVisualizer() {

    // model is chasis
    // model_0 is intake
    /*
    double intakeWidth = 0.5; // Example width of the intake
    double intakeHeight = 0.3; // Example height of the intake
    */
    double amplitud = 0.785;
    double offset = -0.785;
    double anguloRad = (Math.sin(Timer.getFPGATimestamp() * 1.2) * amplitud) + offset;

    intakePose3d = new Pose3d(0.26, 0, 0.255, new Rotation3d(0, anguloRad, 0));
    // intakePose3d = new Pose3d(0.135, 0.0, 0.05, new Rotation3d(0.0,0.0,0.0));

    Logger.recordOutput("ComponentsPoseArray", new Pose3d[] {intakePose3d});
  }
}
