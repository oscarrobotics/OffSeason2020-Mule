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

	private NetworkTableEntry dashboard_wheelPow, dashboard_wheelVolts, dashboard_topWheelRPM, dashboard_bottomWheelRPM, dashboard_leftSliderRPM, dashboard_PID, dashboard_mode, dashboard_FF, dashboard_multiplier;

	PIDController pid = new PIDController(Constants.Shooter.IDLE_kP,0,Constants.Shooter.IDLE_kD);

	private SHOOTER_MODE mode = SHOOTER_MODE.IDLE, lastMode = SHOOTER_MODE.IDLE;

	public ShooterSubsystem () {
		DashboardManager.addTab(this);
		DashboardManager.getTab(this).add(this);

		topWheel = new CANTalonSRX(Constants.Shooter.kTopWheelCANId);
		bottomWheel = new CANTalonSRX(Constants.Shooter.kBottomWheelCANId);

		topWheel.wipeSettings();
		bottomWheel.wipeSettings();

		topWheel.setInverted(true);
		bottomWheel.setInverted(false);

		topWheel.setSensorPhase(true);
		bottomWheel.setSensorPhase(true);


		setCurrentLimit(20);

		NeutralMode neutralMode = NeutralMode.kCoast;
		topWheel.setNeutralMode(neutralMode);
		bottomWheel.setNeutralMode(neutralMode);

//		bottomWheel.follow(topWheel);

		topWheel.getBaseController().configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		bottomWheel.getBaseController().configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

		dashboard_wheelPow = DashboardManager.addTabItem(this, "Power", 0.0);
		dashboard_wheelVolts = DashboardManager.addTabItem(this, "Volts", 0.0);
		dashboard_topWheelRPM = DashboardManager.addTabItem(this, "Top RPM", 0.0);
		dashboard_bottomWheelRPM = DashboardManager.addTabItem(this, "Bottom RPM", 0.0);
		dashboard_leftSliderRPM = DashboardManager.addTabItem(this, "Slider", 0.0);
		dashboard_PID = DashboardManager.addTabItem(this, "PID", 0.0);
		dashboard_mode = DashboardManager.addTabItem(this, "PID Mode", "Idle");
		dashboard_FF = DashboardManager.addTabItem(this, "FeedForward", 0.0);

		setDefaultCommand(new RunEndCommand(this::runShooter, this::stopShooter, this));

		initSuccessful = true;
	}

	@Override
	public void periodic() {
//		updatePIDMode();
	}

	public void setShooterMode (SHOOTER_MODE mode) {
		lastMode = this.mode;
		this.mode = mode;
	}

	@Override
	public void updateDashboardData () {
		dashboard_topWheelRPM.setDouble(topWheel.getSensorVelocity());
		dashboard_bottomWheelRPM.setDouble(bottomWheel.getSensorVelocity());
		dashboard_wheelPow.setDouble(getTopTargetPower());
		dashboard_wheelVolts.setDouble(getTopTargetPower() * 12);
		dashboard_leftSliderRPM.setDouble(getTopTargetRPM());
		dashboard_mode.setString(dashboardGetMode());
		dashboard_PID.setDouble(getTopPIDPow(getTopTargetRPM()));
		dashboard_multiplier.setDouble(getMultiplier());
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

	public void handleTopRPM () {
		double targetRPM = getTopTargetRPM();
		double power = getTopPIDPow(targetRPM);
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
//		double power = 0;
//		if (targetRPM > topWheel.getSensorVelocity() + 1000) {
//			power = Constants.Shooter.ff.calculate(getShooterTargetRPM());//, Constants.Shooter.SPIN_UP_ACCEL);
//		} else if (targetRPM < topWheel.getSensorVelocity() - 1000) {
//			power = Constants.Shooter.ff.calculate(getShooterTargetRPM());//, Constants.Shooter.SPIN_DOWN_ACCEL);
//		} else if (targetRPM > 1000){
//			power = Constants.Shooter.ff.calculate(getShooterTargetRPM());
//		}
//		dashboard_FF.setDouble(power);
		dashboard_PID.setDouble(power);
		topWheel.set(power);
	}

	public void handleRPM() {
		handleTopRPM();
		handleBottomRPM(getMultiplier());
	}

	public void handleBottomRPM(double multiplier) {
		double targetRPM = getTopTargetRPM() * multiplier;
		double power = getBottomPIDPow(targetRPM);
		if (targetRPM > 1000) {
			power += Constants.Shooter.IDLE_kF * (targetRPM / 3000.0);
		}

		bottomWheel.set(power);
	}

	private double getTopPIDPow (double targetRPM) {
		return pid.calculate(topWheel.getSensorVelocity(), targetRPM);
	}

	private double getBottomPIDPow (double targetRPM) {
		return pid.calculate(bottomWheel.getSensorVelocity(), targetRPM);
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
		topWheel.set(getTopTargetPower());
		bottomWheel.set(getBottomTargetPower());
	}

	public double getTopTargetPower () {
		return OscarMath.map(OI.stratComInterface.getLeftSlider(), -1, 1, 0, 1);
	}

	public double getTopTargetRPM () {
		return OscarMath.map(OI.stratComInterface.getLeftSlider(), -1, 1, 0, 15000);
	}

	public double getBottomTargetPower () {
		return OscarMath.map(OI.stratComInterface.getRightSlider(), -1, 1, 0, 1);
	}

	public double getBottomTargetRPM () { return OscarMath.map(OI.stratComInterface.getRightSlider(), -1, 1, 0, 15000); }

	private double getMultiplier() { return OscarMath.map(OI.stratComInterface.getRightSlider(), -1, 1, 0.5, 3); }

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
