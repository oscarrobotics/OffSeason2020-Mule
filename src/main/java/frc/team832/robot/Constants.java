package frc.team832.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;
import frc.team832.lib.motors.DTPowerTrain;
import frc.team832.lib.motors.Gearbox;
import frc.team832.lib.motors.Motors;

public class Constants {
    public static final double AUTODRIVE_KP = 0.0;
    public static final double kDriveWheelDiameter = Units.inchesToMeters(6.1);

    public static final float kDriveGearReduction = 1f / (9f/84f);

    private static final Gearbox driveGearbox = new Gearbox(kDriveGearReduction);
    public static final DTPowerTrain dtPowertrain = new DTPowerTrain(driveGearbox, Motors.NEO, 2, kDriveWheelDiameter);

    public static DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(Units.inchesToMeters(24.0));

    public static Pose2d startPose = new Pose2d(new Translation2d(0, 0), new Rotation2d());

    public static final int leftMasterCANId = 20;
    public static final int leftSlaveCANId = 21;
    public static final int rightMasterCANId = 22;
    public static final int rightSlaveCANId = 23;
}
