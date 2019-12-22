package frc.team832.robot.autonomous;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.team832.robot.Constants;

import java.util.List;

public class Trajectories {


	public static final Trajectory test = TrajectoryGenerator.generateTrajectory(
			Constants.Poses.kZeroZeroPose,
			List.of(),
			new Pose2d(1,0,new Rotation2d(0)),
			Constants.Drivetrain.kTrajectoryConfig
	);

	public static final Trajectory example = TrajectoryGenerator.generateTrajectory(
			// Start at the origin facing the +X direction
			new Pose2d(0, 0, new Rotation2d(0)),
			// Pass through these two interior waypoints, making an 's' curve path
			List.of(
					new Translation2d(2, 1),
					new Translation2d(4, -1)
			),
			// End 3 meters straight ahead of where we started, facing forward
			new Pose2d(6, 0, new Rotation2d(0)),
			// Pass config
			Constants.Drivetrain.kTrajectoryConfig
	);

}
