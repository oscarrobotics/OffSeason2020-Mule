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
import frc.team832.lib.motors.Motors;

public class Constants {
    public static final double AUTODRIVE_KP = 0.0;
    public static final double kDriveWheelDiameter = Units.inchesToMeters(6);

    public static final float kDriveGearReduction = 1f / (9f/84f);

    private static final Gearbox driveGearbox = new Gearbox(kDriveGearReduction);
    public static final DTPowerTrain dtPowertrain = new DTPowerTrain(driveGearbox, Motors.NEO, 2, kDriveWheelDiameter);
	public static DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(Units.inchesToMeters(24.0));

    public static final double kDrive_kS = 0.237;
    public static final double kDrive_kV = 2.5;
    public static final double kDrive_kA = 0.267;

    public static final double kDriveLeft_kP = 0.955;
    public static final double kDriveRight_kP = 0.888;

    public static final SimpleMotorFeedforward kDriveFF = new SimpleMotorFeedforward(kDrive_kS, kDrive_kV, kDrive_kA);

	public static final DifferentialDriveVoltageConstraint kAutoVoltageConstraint =
			new DifferentialDriveVoltageConstraint(kDriveFF, kDriveKinematics, 10);

	public static final TrajectoryConfig kTrajectoryConfig =
			new TrajectoryConfig(1, 1)
					.setKinematics(kDriveKinematics)
					.addConstraint(kAutoVoltageConstraint);


    public static final Pose2d kZeroZeroPose = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
	public static final Pose2d kCenterHABStartPose = new Pose2d(1.7, 4.12, new Rotation2d());
	public static final Pose2d kLeftHABStartPose = new Pose2d(1.7, 3.0, new Rotation2d());
	public static final Pose2d kRightHABStartPose = PathHelper.mirrorPose2d(kLeftHABStartPose);

    public static final int kLeftMasterCANId = 20;
    public static final int kLeftSlaveCANId = 21;
    public static final int kRightMasterCANId = 22;
    public static final int kRightSlaveCANId = 23;
}
