package frc.team832.robot.autonomous;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.team832.lib.motion.PathHelper;
import frc.team832.robot.Constants;

import java.util.List;

public class Path2019 {
	public static final double LEFT_ROCKET_CLOSE_ANGLE =  27.5;
	public static final double RIGHT_ROCKET_CLOSE_ANGLE = -LEFT_ROCKET_CLOSE_ANGLE;
	public static final double LEFT_ROCKET_REAR_ANGLE = 180 - LEFT_ROCKET_CLOSE_ANGLE;
	public static final double RIGHT_ROCKET_REAR_ANGLE = -LEFT_ROCKET_REAR_ANGLE;

	public static final Pose2d CENTER_HAB_START_POSE = new Pose2d(1.7, 4.12, new Rotation2d(0));
	public static final Pose2d LEFT_HAB_START_POSE = new Pose2d(1.7, 3.0, new Rotation2d(0));
	public static final Pose2d RIGHT_HAB_START_POSE = PathHelper.mirrorPose2d(LEFT_HAB_START_POSE);

	public static final Pose2d LEFT_ROCKET_CLOSE_POSE = new Pose2d(16.675, 2.19, Rotation2d.fromDegrees(LEFT_ROCKET_CLOSE_ANGLE));
	public static final Pose2d RIGHT_ROCKET_CLOSE_POSE = PathHelper.mirrorPose2d(LEFT_ROCKET_CLOSE_POSE);

	public static final Pose2d LEFT_CARGOSHIP_FRONT_POSE = new Pose2d(5.4, 3.845, Rotation2d.fromDegrees(0));
	public static final Pose2d RIGHT_CARGOSHIP_FRONT_POSE = PathHelper.mirrorPose2d(LEFT_CARGOSHIP_FRONT_POSE);

	public static final Trajectory LEFT_HAB_LEFT_CARGOSHIP_FRONT_PATH = TrajectoryGenerator.generateTrajectory(LEFT_HAB_START_POSE, List.of(), LEFT_CARGOSHIP_FRONT_POSE, Constants.Drivetrain.kTrajectoryConfig);
	public static final Trajectory RIGHT_HAB_RIGHT_CARGOSHIP_FRONT_PATH = TrajectoryGenerator.generateTrajectory(RIGHT_HAB_START_POSE, List.of(), RIGHT_CARGOSHIP_FRONT_POSE, Constants.Drivetrain.kTrajectoryConfig);
	public static final Trajectory CENTER_HAB_LEFT_CARGOSHIP_FRONT_PATH = TrajectoryGenerator.generateTrajectory(CENTER_HAB_START_POSE, List.of(), LEFT_CARGOSHIP_FRONT_POSE, Constants.Drivetrain.kTrajectoryConfig);
	public static final Trajectory CENTER_HAB_RIGHT_CARGOSHIP_FRONT_PATH = TrajectoryGenerator.generateTrajectory(CENTER_HAB_START_POSE, List.of(), RIGHT_CARGOSHIP_FRONT_POSE, Constants.Drivetrain.kTrajectoryConfig);

	private static final List<Translation2d> LEFT_HAB_LEFT_ROCKET_WAYPOINTS = List.of(
			new Translation2d(3.786, 2.555),
			new Translation2d(4.507, 1.48)
	);
	private static final List<Translation2d> RIGHT_HAB_RIGHT_ROCKET_WAYPOINTS = PathHelper.mirrorTranslation2dList(LEFT_HAB_LEFT_ROCKET_WAYPOINTS);

	public static final Trajectory LEFT_HAB_LEFT_ROCKET_CLOSE_PATH = TrajectoryGenerator.generateTrajectory(LEFT_HAB_START_POSE, LEFT_HAB_LEFT_ROCKET_WAYPOINTS, LEFT_ROCKET_CLOSE_POSE, Constants.Drivetrain.kTrajectoryConfig);
	public static final Trajectory RIGHT_HAB_RIGHT_ROCKET_CLOSE_PATH = TrajectoryGenerator.generateTrajectory(RIGHT_HAB_START_POSE, RIGHT_HAB_RIGHT_ROCKET_WAYPOINTS, RIGHT_ROCKET_CLOSE_POSE, Constants.Drivetrain.kTrajectoryConfig);

}
