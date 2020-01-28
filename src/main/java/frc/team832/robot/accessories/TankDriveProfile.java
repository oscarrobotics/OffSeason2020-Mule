package frc.team832.robot.accessories;

import frc.team832.lib.drive.SmartDiffDrive;
import frc.team832.lib.driverinput.oi.DriveAxesSupplier;
import frc.team832.lib.driverinput.oi.SticksDriverOI;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;

import static frc.team832.robot.Robot.oi;

public class TankDriveProfile {
	public double leftPow;
	public double rightPow;
	public SmartDiffDrive.LoopMode loopMode;
	private double adjustedTurnMultiplier;

	public TankDriveProfile(double leftPow, double rightPow, SmartDiffDrive.LoopMode mode) {
		this.leftPow = leftPow;
		this.rightPow = rightPow;
		this.loopMode = mode;
	}

	public TankDriveProfile() {

	}

	public void calculateTankSpeeds() {
		DriveAxesSupplier axes = oi.driverOI.getGreenbergDriveAxes();
		boolean isPreciseRotate = ((SticksDriverOI) oi.driverOI).rightStick.two.get();
		boolean isRotate =  ((SticksDriverOI) oi.driverOI).rightStick.trigger.get();
		boolean driveStraight = ((SticksDriverOI) oi.driverOI).leftStick.trigger.get();

		if (driveStraight) {
			TankDriveProfile straight = getTankStraightProfile();
			if (isPreciseRotate || isRotate) {
				TankDriveProfile rotate;
				if (isPreciseRotate) {
					rotate = getTankPreciseRotateProfile();
				} else {
					rotate = getTankRotateProfile();
				}
				setSpeeds(straight.leftPow + rotate.leftPow, straight.rightPow + rotate.rightPow, SmartDiffDrive.LoopMode.PERCENTAGE);
			} else {
				setSpeeds(straight.leftPow, straight.rightPow, SmartDiffDrive.LoopMode.PERCENTAGE);
			}
		} else {
			if (isPreciseRotate) {
				TankDriveProfile rotateOnCenter = getTankRotateOnCenterProfile();
				setSpeeds(rotateOnCenter.leftPow, rotateOnCenter.rightPow, SmartDiffDrive.LoopMode.PERCENTAGE);
			} else {
				TankDriveProfile profile = getTankNormalProfile();
				setSpeeds(profile.leftPow, profile.rightPow, SmartDiffDrive.LoopMode.PERCENTAGE);
			}
		}
	}

	public TankDriveProfile getTankStraightProfile() {
		DriveAxesSupplier axes = oi.driverOI.getGreenbergDriveAxes();
		double rightStick = axes.getRightY();
		double leftStick = axes.getLeftY();
		double power;
		if (((SticksDriverOI) oi.driverOI).leftStick.trigger.get() && ((SticksDriverOI) oi.driverOI).rightStick.trigger.get()) {
			power = (OscarMath.signumPow(rightStick * Constants.Drivetrain.StickDriveMultiplier, 2) + OscarMath.signumPow(leftStick * Constants.Drivetrain.StickDriveMultiplier, 2)) / 2;
		} else if (((SticksDriverOI) oi.driverOI).rightStick.trigger.get()) {
			power = OscarMath.signumPow(rightStick * Constants.Drivetrain.StickDriveMultiplier, 2);
		} else {
			power = (OscarMath.signumPow(leftStick * Constants.Drivetrain.StickDriveMultiplier, 2));
		}

		return new TankDriveProfile(power, power, SmartDiffDrive.LoopMode.PERCENTAGE);
	}

	public TankDriveProfile getTankNormalProfile() {
		double rightPower = 0;
		double leftPower = 0;
		DriveAxesSupplier axes = oi.driverOI.getGreenbergDriveAxes();
		double rightStick = axes.getRightY();
		double leftStick = axes.getLeftY();

		rightPower = OscarMath.signumPow(rightStick * Constants.Drivetrain.StickDriveMultiplier, 2);
		leftPower = OscarMath.signumPow(leftStick * Constants.Drivetrain.StickDriveMultiplier, 2);

		return new TankDriveProfile(leftPower, rightPower, SmartDiffDrive.LoopMode.PERCENTAGE);
	}

	public TankDriveProfile getTankRotateProfile() {
		double rightPower = 0;
		double leftPower = 0;
		DriveAxesSupplier axes = oi.driverOI.getGreenbergDriveAxes();

		double power = OscarMath.signumPow(axes.getRightX() * Constants.Drivetrain.StickRotateMultiplier, 2);

		adjustedTurnMultiplier = OscarMath.map(Math.abs(power), 0.0, 1.0, Constants.Drivetrain.StickRotateMultiplier, Constants.Drivetrain.StickRotateMultiplier * 2);

		rightPower = power * adjustedTurnMultiplier;
		leftPower = -power * adjustedTurnMultiplier;

		return new TankDriveProfile(leftPower, rightPower, SmartDiffDrive.LoopMode.PERCENTAGE);
	}

	TankDriveProfile getTankPreciseRotateProfile() {
		double rightPower = 0;
		double leftPower = 0;
		DriveAxesSupplier axes = oi.driverOI.getGreenbergDriveAxes();
		rightPower = -OscarMath.signumPow(axes.getRotation() * Constants.Drivetrain.StickRotateMultiplier, 2);
		leftPower = OscarMath.signumPow(axes.getRotation() * Constants.Drivetrain.StickRotateMultiplier, 2);


		return new TankDriveProfile(leftPower, rightPower, SmartDiffDrive.LoopMode.PERCENTAGE);
	}

	public TankDriveProfile getTankRotateOnCenterProfile() {
		DriveAxesSupplier axes = oi.driverOI.getGreenbergDriveAxes();
		double rotation = OscarMath.signumPow(axes.getRotation() * Constants.Drivetrain.StickRotateOnCenterMultiplier, 3);

		return new TankDriveProfile(-rotation, rotation, SmartDiffDrive.LoopMode.PERCENTAGE);
	}

	private void setSpeeds(double leftPow, double rightPow, SmartDiffDrive.LoopMode mode) {
		this.leftPow = leftPow;
		this.rightPow = rightPow;
		this.loopMode = mode;
	}
}

