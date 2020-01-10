package frc.team832.robot;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import frc.team832.lib.motion.PathHelper;
import frc.team832.lib.motors.DTPowerTrain;
import frc.team832.lib.motors.Gearbox;
import frc.team832.lib.motors.Motor;

public class Constants {

	public static class Poses {
		public static final Pose2d kZeroZeroPose = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
		public static final Pose2d kCenterHABStartPose = new Pose2d(1.7, 4.12, new Rotation2d());
		public static final Pose2d kRightHABStartPose = new Pose2d(1.7, 3.0, new Rotation2d());
		public static final Pose2d kLeftHABStartPose = PathHelper.mirrorPose2d(kRightHABStartPose);
	}

	public static class Drivetrain {
		public static final double kDriveWheelDiameter = Units.inchesToMeters(6);

		public static final float kDriveGearReduction = 1f / (9f/84f);

		private static final Gearbox driveGearbox = new Gearbox(kDriveGearReduction);
		public static final DTPowerTrain dtPowertrain = new DTPowerTrain(driveGearbox, Motor.kNEO, 2, kDriveWheelDiameter);
		public static DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(Units.inchesToMeters(24.0));

		private static final double kDrive_kS = 0.237;
		private static final double kDrive_kV = 2.5;
		private static final double kDrive_kA = 0.267;

		private static final SimpleMotorFeedforward kDriveFF = new SimpleMotorFeedforward(kDrive_kS, kDrive_kV, kDrive_kA);

		private static final double kLeftDrive_kS = 0.242;
		private static final double kLeftDrive_kV = 2.5;
		private static final double kLeftDrive_kA = 0.258;

		public static final SimpleMotorFeedforward kLeftDriveFF = new SimpleMotorFeedforward(kLeftDrive_kS, kLeftDrive_kV, kLeftDrive_kA);

		private static final double kRightDrive_kS = 0.226;
		private static final double kRightDrive_kV = 2.5;
		private static final double kRightDrive_kA = 0.241;

		public static final SimpleMotorFeedforward kRightDriveFF = new SimpleMotorFeedforward(kRightDrive_kS, kRightDrive_kV, kRightDrive_kA);

		public static final double kDriveLeft_kP = 0; // 0.995
		public static final double kDriveRight_kP = 0; // 0.888

		public static final DifferentialDriveVoltageConstraint kAutoVoltageConstraint =
				new DifferentialDriveVoltageConstraint(kDriveFF, kDriveKinematics, 10);

		public static final TrajectoryConfig kTrajectoryConfig =
				new TrajectoryConfig(3, 6)
						.setKinematics(kDriveKinematics)
						.addConstraint(kAutoVoltageConstraint);

		public static final TrajectoryConfig kSlowTrajConfig =
				new TrajectoryConfig(1, 1)
						.setKinematics(kDriveKinematics)
						.addConstraint(kAutoVoltageConstraint);

		public static final TrajectoryConfig kMedTrajConfig =
				new TrajectoryConfig(2, 2)
						.setKinematics(kDriveKinematics)
						.addConstraint(kAutoVoltageConstraint);

		public static final TrajectoryConfig kFastTrajConfig =
				new TrajectoryConfig(3, 6)
						.setKinematics(kDriveKinematics)
						.addConstraint(kAutoVoltageConstraint);

		public static final int kLeftMasterCANId = 20;
		public static final int kLeftSlaveCANId = 21;
		public static final int kRightMasterCANId = 22;
		public static final int kRightSlaveCANId = 23;

		public static final double visionMoveKp = .25;
		public static final double visionRotKp = .25/90;
	}

	public static class Shooter {
		public static final int kTopWheelCANId = 1;
		public static final int kBottomWheelCANId = 2;

		public static double SPIN_UP_kP = 0;
		public static double SPIN_UP_kD = 0;
		public static double SPIN_UP_kF = 0;

		public static double SHOOTING_kP = 0;
		public static double SHOOTING_kD = 0;
		public static double SHOOTING_kF = 0;

		public static double SPIN_DOWN_kP = 0;
		public static double SPIN_DOWN_kD = 0;
		public static double SPIN_DOWN_kF = 0;
	}
}
