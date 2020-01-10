package frc.team832.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.RunEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.driverstation.dashboard.DashboardUpdatable;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol2.vendor.CANTalonSRX;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;
import frc.team832.robot.OI;

public class ShooterSubsystem extends SubsystemBase implements DashboardUpdatable {
	private CANTalonSRX topWheel, bottomWheel;
	private boolean initSuccessful = false;

	private NetworkTableEntry dashboard_wheelPow, dashboard_wheelVolts;

	PIDController pid = new PIDController(Constants.Shooter.SPIN_UP_kP,0,Constants.Shooter.SPIN_UP_kD);

	private SHOOTER_MODE mode = SHOOTER_MODE.IDLE, lastMode = SHOOTER_MODE.IDLE;

	public ShooterSubsystem () {
		DashboardManager.addTab(this);
		DashboardManager.getTab(this).add(this);

		topWheel = new CANTalonSRX(Constants.Shooter.kTopWheelCANId);
		bottomWheel = new CANTalonSRX(Constants.Shooter.kBottomWheelCANId);

		topWheel.wipeSettings();
		bottomWheel.wipeSettings();

		topWheel.setInverted(true);
		bottomWheel.setInverted(true);

		setCurrentLimit(40);

		NeutralMode neutralMode = NeutralMode.kCoast;
		topWheel.setNeutralMode(neutralMode);
		bottomWheel.setNeutralMode(neutralMode);

		bottomWheel.follow(topWheel);

		dashboard_wheelPow = DashboardManager.addTabItem(this, "Power", 0.0);
		dashboard_wheelVolts = DashboardManager.addTabItem(this, "Volts", 0.0);

		setDefaultCommand(new RunEndCommand(this::runShooter, this::stopShooter, this));

		initSuccessful = true;
	}

	@Override
	public void periodic() {
		if(lastMode != mode){
			if (mode == SHOOTER_MODE.SHOOTING){
				pid.setPID(Constants.Shooter.SHOOTING_kP, 0, Constants.Shooter.SHOOTING_kD);
			} else if (mode == SHOOTER_MODE.SPINNING_UP) {
				pid.setPID(Constants.Shooter.SPIN_UP_kP, 0, Constants.Shooter.SPIN_UP_kD);
			} else if (mode == SHOOTER_MODE.SPINNING_DOWN) {
				pid.setPID(Constants.Shooter.SPIN_DOWN_kP, 0, Constants.Shooter.SPIN_DOWN_kD);
			} else {
				pid.setPID(0,0,0);
			}
		}
	}

	public void setShooterMode (SHOOTER_MODE mode) {
		lastMode = this.mode;
		this.mode = mode;
	}

	public SHOOTER_MODE getMode () {
		return mode;
	}

	private void updatePIDMode () {
		if (mode == SHOOTER_MODE.SHOOTING){
			pid.setPID(Constants.Shooter.SHOOTING_kP, 0, Constants.Shooter.SHOOTING_kD);
		} else if (mode == SHOOTER_MODE.SPINNING_UP) {
			pid.setPID(Constants.Shooter.SPIN_UP_kP, 0, Constants.Shooter.SPIN_UP_kD);
		}
	}

	@Override
	public String getDashboardTabName () {
		return "Shooter";
	}

	@Override
	public void updateDashboardData() {
		dashboard_wheelPow.setDouble(getShooterPower());
		dashboard_wheelVolts.setDouble(getShooterPower() * 12);
	}

	public enum SHOOTER_MODE {
		SPINNING_UP,
		SPINNING_DOWN,
		SHOOTING,
		IDLE
	}

	public void setShooterPower() {
		topWheel.set(getShooterPower());
	}

	public double getShooterPower() {
		return OscarMath.map(OI.stratComInterface.getLeftSlider(), -1, 1, 0, 0.9);
	}

	public void runShooter() {
		setShooterPower();
	}

	public void stopShooter() {
		topWheel.set(0);
	}

	public void spinUp() {
		setRPM(dashboard_wheelPow.getDouble(0));
		setShooterMode(SHOOTER_MODE.SPINNING_UP);
	}

	public void setRPM(double rpm) {
		topWheel.setVelocity(rpm);
	}

	public void spinDown() {
		setRPM(0);
		setShooterMode(SHOOTER_MODE.SPINNING_DOWN);
	}

	public void setCurrentLimit(int limit) {
//		topWheel.setC(limit);
//		bottomWheel.setPeakCurrentLimit(limit);
	}

	public boolean passedInit() {
		return initSuccessful;
	}

}
