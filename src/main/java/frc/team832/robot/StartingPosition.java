package frc.team832.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;

public enum StartingPosition {
	kZeroZero(Constants.Poses.kZeroZeroPose),
	kHAB1Left(Constants.Poses.kLeftHABStartPose),
	kHAB1Center(Constants.Poses.kCenterHABStartPose),
	kHAB1Right(Constants.Poses.kRightHABStartPose);

	public final Pose2d poseMeters;

	StartingPosition (Pose2d pose) {
		poseMeters = pose;
	}
}
