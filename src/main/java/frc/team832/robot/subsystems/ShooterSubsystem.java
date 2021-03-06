package frc.team832.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.RunEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.driverstation.dashboard.DashboardUpdatable;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol.vendor.CANTalon;
import frc.team832.robot.Constants;

public class ShooterSubsystem extends SubsystemBase implements DashboardUpdatable {
	private CANTalon topWheel, bottomWheel;
	private boolean initSuccessful = false;

	private NetworkTableEntry dashboard_wheelRPM;

	PIDController pid = new PIDController(Constants.Shooter.SPIN_UP_kP,0,Constants.Shooter.SPIN_UP_kD);

	private SHOOTER_MODE mode = SHOOTER_MODE.IDLE, lastMode = SHOOTER_MODE.IDLE;

	public ShooterSubsystem () {
		topWheel = new CANTalon(Constants.Shooter.kTopWheelCANId);
		bottomWheel = new CANTalon(Constants.Shooter.kBottomWheelCANId);

		topWheel.resetSettings();
		bottomWheel.resetSettings();

		topWheel.setInverted(false);
		bottomWheel.setInverted(true);

		setCurrentLimit(40);

		NeutralMode neutralMode = NeutralMode.kCoast;
		topWheel.setNeutralMode(neutralMode);
		bottomWheel.setNeutralMode(neutralMode);

		dashboard_wheelRPM = DashboardManager.addTabItem(this, "Speeds/LeftFPS", 0.0);

		setDefaultCommand(new RunEndCommand(this::spinUp, this::spinDown, this));

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

		if (Math.abs(dashboard_wheelRPM.getDouble(0) - topWheel.getSensorVelocity()) > 1000) {
			setRPM(dashboard_wheelRPM.getDouble(0));
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
	public void updateDashboardData () {
		dashboard_wheelRPM.setDouble(topWheel.getSensorVelocity());
	}

	public enum SHOOTER_MODE {
		SPINNING_UP,
		SPINNING_DOWN,
		SHOOTING,
		IDLE
	}

	public void spinUp() {
		setRPM(dashboard_wheelRPM.getDouble(0));
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
		topWheel.setPeakCurrentLimit(limit);
		bottomWheel.setPeakCurrentLimit(limit);
	}

	public boolean passedInit() {
		return initSuccessful;
	}

}
