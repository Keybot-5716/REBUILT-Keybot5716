package frc.robot.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;

public class AutoRightTrench extends AutoBuilder {
  private final PathPlannerPath right_to_center;
  private final PathPlannerPath right_center_intake;

  public AutoRightTrench() {
    right_to_center = loadPath("RIGHT_TRENCH_TO_CENTER");
    right_center_intake = loadPath("RIGHTCENTER_INTAKE");

    addCommands(Commands.deadline(Commands.sequence(new PathPlannerAuto("RIGHT_TRENCH_AUTO"))));
  }

  @Override
  public List<Pose2d> getPathPoses() {
    return getPathPosesList(right_to_center, right_center_intake);
  }

  @Override
  public Pose2d getStartingPose() {
    if (right_to_center != null) {
      return right_to_center.getStartingDifferentialPose();
    }

    DriverStation.reportError("Path is null", true);
    return new Pose2d();
  }
}
