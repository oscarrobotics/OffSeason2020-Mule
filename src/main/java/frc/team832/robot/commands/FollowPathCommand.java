package frc.team832.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.team832.robot.Constants;
import frc.team832.robot.Robot;

public class FollowPathCommand extends RamseteCommand {
	private static final RamseteController ramseteController = new RamseteController();
	private static final PIDController leftDrivePIDController = new PIDController(Constants.kDriveLeft_kP, 0, 0);
	private static final PIDController rightDrivePIDController = new PIDController(Constants.kDriveRight_kP, 0, 0);

	public FollowPathCommand (Trajectory trajectory) {
		super(trajectory, Robot.drivetrain::getLatestPose, ramseteController, Constants.kDriveFF, Constants.kDriveKinematics, Robot.drivetrain::getWheelSpeeds, leftDrivePIDController, rightDrivePIDController, Robot.drivetrain::setWheelVolts, Robot.drivetrain);
	}
}
