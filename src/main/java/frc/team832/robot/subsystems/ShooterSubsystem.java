package frc.team832.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
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

	private NetworkTableEntry dashboard_wheelPow, dashboard_wheelVolts, dashboard_wheelRPM, dashboard_sliderRPM, dashboard_PID, dashboard_mode;

	PIDController pid = new PIDController(Constants.Shooter.SPIN_UP_kP,0,Constants.Shooter.SPIN_UP_kD);

	private SHOOTER_MODE mode = SHOOTER_MODE.IDLE, lastMode = SHOOTER_MODE.IDLE;

	public ShooterSubsystem () {
		DashboardManager.addTab(this);
		DashboardManager.getTab(this).add(this);

		topWheel = new CANTalonSRX(Constants.Shooter.kTopWheelCANId);
		bottomWheel = new CANTalonSRX(Constants.Shooter.kBottomWheelCANId);

		topWheel.wipeSettings();
		bottomWheel.wipeSettings();

		topWheel.setInverted(false);
		bottomWheel.setInverted(false);

		setCurrentLimit(40);

		NeutralMode neutralMode = NeutralMode.kCoast;
		topWheel.setNeutralMode(neutralMode);
		bottomWheel.setNeutralMode(neutralMode);

		bottomWheel.follow(topWheel);

		topWheel.getBaseController().configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

		dashboard_wheelPow = DashboardManager.addTabItem(this, "Power", 0.0);
		dashboard_wheelVolts = DashboardManager.addTabItem(this, "Volts", 0.0);
		dashboard_wheelRPM = DashboardManager.addTabItem(this, "RPM", 0.0);
		dashboard_sliderRPM = DashboardManager.addTabItem(this, "Slider", 0.0);
		dashboard_PID = DashboardManager.addTabItem(this, "PID", 0.0);
		dashboard_mode = DashboardManager.addTabItem(this, "PID Mode", "Idle");

		setDefaultCommand(new RunEndCommand(this::runShooter, this::stopShooter, this));

		initSuccessful = true;
	}

	@Override
	public void periodic() {
		updatePIDMode();
	}

	public void setShooterMode (SHOOTER_MODE mode) {
		lastMode = this.mode;
		this.mode = mode;
	}

	@Override
	public void updateDashboardData () {
		dashboard_wheelRPM.setDouble(topWheel.getSensorVelocity());
		dashboard_wheelPow.setDouble(getShooterTargetPower());
		dashboard_wheelVolts.setDouble(getShooterTargetPower() * 12);
		dashboard_sliderRPM.setDouble(getShooterTargetRPM());
		dashboard_mode.setString(dashboardGetMode());
		dashboard_PID.setDouble(getPIDPow(getShooterTargetRPM()));
	}
	
	public SHOOTER_MODE getMode () {
		return mode;
	}

	private String dashboardGetMode() {
		switch (mode) {
			case IDLE:
				return "Idle";
			case SHOOTING:
				return "Shooting";
			case SPINNING_UP:
				return "Spin Up";
			case SPINNING_DOWN:
				return "Spin Down";

		}
		return null;
	}

	private void updatePIDMode () {
		if (mode == SHOOTER_MODE.SPINNING_UP){
			pid.setPID(Constants.Shooter.SPIN_UP_kP, 0, Constants.Shooter.SPIN_UP_kD);
		} else if (mode == SHOOTER_MODE.SPINNING_DOWN) {
			pid.setPID(Constants.Shooter.SPIN_DOWN_kP, 0, Constants.Shooter.SPIN_DOWN_kD);
		} else if (mode == SHOOTER_MODE.SHOOTING){
			pid.setPID(Constants.Shooter.SHOOTING_kP, 0, Constants.Shooter.SHOOTING_kD);
		} else {
			pid.setPID(Constants.Shooter.IDLE_kP, 0, Constants.Shooter.IDLE_kD);
		}
	}

	public void handleRPM () {
		double targetRPM = getShooterTargetRPM();
		double power = getPIDPow(targetRPM);
		if (targetRPM > topWheel.getSensorVelocity() + 1000) {
			setShooterMode(SHOOTER_MODE.SPINNING_UP);
			power += Constants.Shooter.SPIN_UP_kF;
		} else if (targetRPM < topWheel.getSensorVelocity() - 1000) {
			setShooterMode(SHOOTER_MODE.SPINNING_DOWN);
			power += Constants.Shooter.SPIN_DOWN_kF;
		} else if (targetRPM > 1000){
			setShooterMode(SHOOTER_MODE.IDLE);
			power += Constants.Shooter.IDLE_kF * (targetRPM / 3000.0);
		}
		topWheel.set(power);
	}

	private double getPIDPow(double targetRPM) {
		return pid.calculate(topWheel.getSensorVelocity(), targetRPM);
	}

	public enum SHOOTER_MODE {
		SPINNING_UP,
		SPINNING_DOWN,
		SHOOTING,
		IDLE
	}


	@Override
	public String getDashboardTabName () {
		return "Shooter";
	}
	
	public void setShooterPower() {
		topWheel.set(getShooterTargetPower());
	}

	public double getShooterTargetPower () {
		return OscarMath.map(OI.stratComInterface.getLeftSlider(), -1, 1, 0, 1);
	}

	public double getShooterTargetRPM () {
		return OscarMath.map(OI.stratComInterface.getLeftSlider(), -1, 1, 0, 18000);
	}

	public void runShooter() {
//		setShooterPower();
		handleRPM();
	}

	public void stopShooter() {
		topWheel.set(0);
	}

	public void setCurrentLimit(int limit) {
		topWheel.limitInputCurrent(limit);
		bottomWheel.limitInputCurrent(limit);
	}

	public boolean passedInit() {
		return initSuccessful;
	}

}
