package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class DriveTelemetry {
    private  final double maxSpeed;
    private final String prefix = "Drive/Telemetry/";

    private final Field2d field = new Field2d();

    public DriveTelemetry(double maxSpeed) {
        this.maxSpeed = maxSpeed;
        SmartDashboard.putData(field);
    }

    public void telemeterize(SwerveDriveState state) {
        if(state == null || state.Pose == null || state.ModuleStates == null) return;

        Pose2d pose = state.Pose;
        Logger.recordOutput(prefix+"Pose", pose);

        Pose3d pose3d = new Pose3d(pose.getX(), pose.getY(), 0.0, new Rotation3d(0.0,0.0,pose.getRotation().getRadians()));
        Logger.recordOutput(prefix+"Pose3d", pose3d);

        if(DriverStation.isDisabled() || Robot.isSimulation()) {
            field.setRobotPose(pose);
        }
    }
}
