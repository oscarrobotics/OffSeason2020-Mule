package frc.team832.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.RunEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.drive.SmartDifferentialDrive;
import frc.team832.lib.driverinput.oi.DriveAxesSupplier;
import frc.team832.lib.driverinput.oi.SticksDriverOI;
import frc.team832.lib.driverinput.oi.XboxDriverOI;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.driverstation.dashboard.DashboardUpdatable;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol.vendor.CANSparkMax;
import frc.team832.lib.motors.Motors;
import frc.team832.lib.sensors.NavXMicro;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;

import static com.revrobotics.CANSparkMaxLowLevel.*;
import static frc.team832.robot.Robot.oi;

@SuppressWarnings({"FieldCanBeLocal", "WeakerAccess"})
public class DrivetrainSubsystem extends SubsystemBase implements DashboardUpdatable {
    private final CANSparkMax leftMaster, leftSlave, rightMaster, rightSlave;

    private NetworkTableEntry dashboard_poseX, dashboard_poseY, dashboard_poseHeadingDegrees, dashboard_poseHeadingRadians, dahsboard_navXYaw,
            dashboard_rightDistance, dashboard_leftDistance, dashboard_rightWheelSpeedMPS, dashboard_leftWheelSpeedMPS,
            dashboard_rightOutputVolts, dashboard_leftOutputVolts;

    private NetworkTable falconTable = NetworkTableInstance.getDefault().getTable("Live_Dashboard");
    private NetworkTableEntry falconPoseXEntry = falconTable.getEntry("robotX");
    private NetworkTableEntry falconPoseYEntry = falconTable.getEntry("robotY");
    private NetworkTableEntry falconPoseHeadingEntry = falconTable.getEntry("robotHeading");
    private NetworkTableEntry falconIsPathingEntry = falconTable.getEntry("isFollowingPath");
    private NetworkTableEntry falconPathXEntry = falconTable.getEntry("pathX");
    private NetworkTableEntry falconPathYEntry = falconTable.getEntry("pathY");
    private NetworkTableEntry falconPathHeadingEntry = falconTable.getEntry("pathHeading");

    private final SmartDifferentialDrive diffDrive;
    public DifferentialDriveOdometry driveOdometry;
    public Pose2d pose = new Pose2d();
    public NavXMicro navX;

    private boolean initPassed = true;

    public DrivetrainSubsystem() {
        leftMaster = new CANSparkMax(Constants.kLeftMasterCANId, MotorType.kBrushless);
        leftSlave = new CANSparkMax(Constants.kLeftSlaveCANId, MotorType.kBrushless);
        rightMaster = new CANSparkMax(Constants.kRightMasterCANId, MotorType.kBrushless);
        rightSlave = new CANSparkMax(Constants.kRightSlaveCANId, MotorType.kBrushless);

        resetMCSettings();

        if (!(leftMaster.getInputVoltage() > 0)) initPassed = false;
        if (!(leftSlave.getInputVoltage() > 0)) initPassed = false;
        if (!(rightMaster.getInputVoltage() > 0)) initPassed = false;
        if (!(rightSlave.getInputVoltage() > 0)) initPassed = false;

        NeutralMode neutralMode = NeutralMode.kBrake;

        leftMaster.setNeutralMode(neutralMode);
        leftSlave.setNeutralMode(neutralMode);
        rightMaster.setNeutralMode(neutralMode);
        rightSlave.setNeutralMode(neutralMode);

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        resetMCCurrentLimit();

        rightMaster.setInverted(true);
        rightSlave.setInverted(false);
        leftMaster.setInverted(true);
        leftSlave.setInverted(false);

        diffDrive = new SmartDifferentialDrive(leftMaster, rightMaster, Motors.NEO.freeSpeed - 1000);

        navX = new NavXMicro(NavXMicro.NavXPort.I2C_onboard);
        navX.init();
        navX.zero();

        driveOdometry = new DifferentialDriveOdometry(getDriveHeading() , Constants.kZeroZeroPose);

        DashboardManager.addTab(this);
        dashboard_poseX = DashboardManager.addTabItem(this, "Pose/X", 0.0);
        dashboard_poseY = DashboardManager.addTabItem(this, "Pose/Y", 0.0);
        dashboard_poseHeadingDegrees = DashboardManager.addTabItem(this, "Pose/HeadingDeg", 0.0);
        dashboard_poseHeadingRadians = DashboardManager.addTabItem(this, "Pose/HeadingRad", 0.0);
        dahsboard_navXYaw = DashboardManager.addTabItem(this, "NavX Yaw", 0.0);
        dashboard_leftDistance = DashboardManager.addTabItem(this, "LeftDriveDistance", 0.0);
        dashboard_rightDistance = DashboardManager.addTabItem(this, "RightDriveDistance", 0.0);
        dashboard_leftOutputVolts = DashboardManager.addTabItem(this, "Left Volts", 0.0);
        dashboard_rightOutputVolts = DashboardManager.addTabItem(this, "Right Volts", 0.0);
        dashboard_leftWheelSpeedMPS = DashboardManager.addTabItem(this, "Left Wheel Speed", 0.0);
        dashboard_rightWheelSpeedMPS = DashboardManager.addTabItem(this, "Right Wheel Speed", 0.0);

        resetPose();

        setDefaultCommand(new RunEndCommand(this::drive, diffDrive::stopMotor, this));
    }

    public void periodic() {

    }

    public void resetMCSettings () {
        leftMaster.resetSettings();
        rightMaster.resetSettings();
        leftSlave.resetSettings();
        rightSlave.resetSettings();
    }

    public void resetMCCurrentLimit () {
        rightMaster.setPeakCurrentLimit(40);
        leftMaster.setPeakCurrentLimit(40);
        rightSlave.setPeakCurrentLimit(40);
        leftSlave.setPeakCurrentLimit(40);
    }

    public void setMCCurrentLimit (int currentLimit) {
        rightMaster.setPeakCurrentLimit(currentLimit);
        leftMaster.setPeakCurrentLimit(currentLimit);
        rightSlave.setPeakCurrentLimit(currentLimit);
        leftSlave.setPeakCurrentLimit(currentLimit);
    }

    public boolean passedInit() {
        return initPassed;
    }

    private void updatePose() {
        pose = driveOdometry.update(getDriveHeading(), getLeftDistanceMeters(), getRightDistanceMeters());
    }

    public void updateDashboardPose() {

        updatePose();

        var translation = pose.getTranslation();
        var poseX = translation.getX();
        var poseY = translation.getY();
        var heading = pose.getRotation();

        falconPoseXEntry.setDouble(Units.metersToFeet(poseX));
        falconPoseYEntry.setDouble(Units.metersToFeet(poseY));
        falconPoseHeadingEntry.setDouble(heading.getRadians());

        dashboard_poseX.setDouble(OscarMath.round(poseX, 3));
        dashboard_poseY.setDouble(OscarMath.round(poseY, 3));

        dashboard_poseHeadingDegrees.setDouble(OscarMath.round(heading.getDegrees(), 2));
        dashboard_poseHeadingRadians.setDouble(OscarMath.round(heading.getRadians(), 3));
    }

    public Rotation2d getDriveHeading() {
        return Rotation2d.fromDegrees(-navX.getYaw());
    }

    public double getRightDistanceMeters () {
        return Constants.dtPowertrain.calculateWheelDistanceMeters(-rightMaster.getSensorPosition());
    }

    public double getLeftDistanceMeters () {
        return Constants.dtPowertrain.calculateWheelDistanceMeters(leftMaster.getSensorPosition());
    }

    public double getRightVelocityMetersPerSec () {
        return Constants.dtPowertrain.calculateMetersPerSec(rightMaster.getSensorVelocity());
    }

    public double getLeftVelocityMetersPerSec () {
        return Constants.dtPowertrain.calculateMetersPerSec(leftMaster.getSensorVelocity());
    }

    public void resetPose() {
        pose = Constants.kZeroZeroPose;
        resetPose(pose);
    }

    public void resetPose(Pose2d pose) {
        resetEncoders();
        this.pose = pose;
        navX.zero();
        driveOdometry.resetPosition(this.pose, getDriveHeading());
    }

    public void resetEncoders() {
        leftMaster.rezeroSensor();
        rightMaster.rezeroSensor();
    }

    public void drive() {
        double rightAxis = 0;
        double leftAxis = 0;
        if (oi.driverOI instanceof XboxDriverOI) {
            DriveAxesSupplier axes = oi.driverOI.getArcadeDriveAxes();
            rightAxis = OscarMath.signumPow(-axes.getRight(), 3);
            leftAxis = OscarMath.signumPow(axes.getLeft(), 3);
            diffDrive.arcadeDrive(rightAxis, leftAxis, SmartDifferentialDrive.LoopMode.PERCENTAGE);
        } else if (oi.driverOI instanceof SticksDriverOI) {
            DriveAxesSupplier axes = oi.driverOI.getTankDriveAxes();
            if (((SticksDriverOI) oi.driverOI).rightStick.two.get()) {
                double rotation = OscarMath.signumPow(axes.getRotation(), 2);
                diffDrive.arcadeDrive(rotation, 0.0, SmartDifferentialDrive.LoopMode.PERCENTAGE);
            } else {
                if (((SticksDriverOI) oi.driverOI).leftStick.trigger.get() || ((SticksDriverOI) oi.driverOI).rightStick.trigger.get()) {
                    double power = (OscarMath.signumPow(axes.getRight(), 2) + OscarMath.signumPow(axes.getLeft(), 2)) / 2;
                    rightAxis = power;
                    leftAxis = power;
                } else {
                    rightAxis = OscarMath.signumPow(axes.getRight(), 3);
                    leftAxis = OscarMath.signumPow(axes.getLeft(), 3);
                }
                diffDrive.tankDrive(leftAxis, rightAxis, SmartDifferentialDrive.LoopMode.PERCENTAGE);
            }
        }
    }

    @Override
    public String getDashboardTabName () {
        return "Drivetrain";
    }

    @Override
    public void updateDashboardData () {
        updateDashboardPose();
        dashboard_leftDistance.setDouble(getLeftDistanceMeters());
        dashboard_rightDistance.setDouble(getRightDistanceMeters());
        dahsboard_navXYaw.setDouble(navX.getYaw());
        dashboard_leftWheelSpeedMPS.setDouble(getLeftVelocityMetersPerSec());
        dashboard_rightWheelSpeedMPS.setDouble(getRightVelocityMetersPerSec());
    }

    public Pose2d getLatestPose () {
        updatePose();
        return pose;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds () {
        return new DifferentialDriveWheelSpeeds(getLeftVelocityMetersPerSec(), getRightVelocityMetersPerSec());
    }

    public void setWheelVolts (Double leftVolts, Double rightVolts) {
        double leftBusVoltage = leftMaster.getInputVoltage();
        double rightBusVoltage = rightMaster.getInputVoltage();

        double leftOutput = Math.abs(leftVolts / leftBusVoltage) * Math.signum(leftVolts);
        double rightOutput = Math.abs(rightVolts / rightBusVoltage) * Math.signum(rightVolts);

        dashboard_rightOutputVolts.setDouble(rightOutput);
        dashboard_leftOutputVolts.setDouble(leftOutput);

        leftMaster.set(leftOutput);
        rightMaster.set(-rightOutput);

    }
}
