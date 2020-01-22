package frc.team832.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.RunEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.drive.SmartDiffDrive;
import frc.team832.lib.driverinput.oi.DriveAxesSupplier;
import frc.team832.lib.driverinput.oi.SticksDriverOI;
import frc.team832.lib.driverinput.oi.XboxDriverOI;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.driverstation.dashboard.DashboardUpdatable;
import frc.team832.lib.driverstation.dashboard.DashboardWidget;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.lib.motors.Motor;
import frc.team832.lib.sensors.NavXMicro;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;
import frc.team832.robot.accessories.TankDriveProfile;

import static com.revrobotics.CANSparkMaxLowLevel.*;
import static frc.team832.robot.Robot.oi;
import static frc.team832.robot.autonomous.SequenceOptions.*;

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

    private final SmartDiffDrive diffDrive;
    public DifferentialDriveOdometry driveOdometry;
    public Pose2d pose = new Pose2d();
    public NavXMicro navX;

    private boolean initPassed = true;

        private Pose2d startingPose = StartingPosition.kHAB1Center.poseMeters;
        private SendableChooser<StartingPosition> startPoseChooser;

    public TankDriveProfile tankProfile = new TankDriveProfile();

    public DrivetrainSubsystem() {
        leftMaster = new CANSparkMax(Constants.Drivetrain.kLeftMasterCANId, MotorType.kBrushless);
        leftSlave = new CANSparkMax(Constants.Drivetrain.kLeftSlaveCANId, MotorType.kBrushless);
        rightMaster = new CANSparkMax(Constants.Drivetrain.kRightMasterCANId, MotorType.kBrushless);
        rightSlave = new CANSparkMax(Constants.Drivetrain.kRightSlaveCANId, MotorType.kBrushless);

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

        setMCCurrentLimit(60);

        rightMaster.setInverted(true);
        rightSlave.setInverted(false);
        leftMaster.setInverted(true);
        leftSlave.setInverted(false);

        diffDrive = new SmartDiffDrive(leftMaster, rightMaster, (int)Motor.kNEO.freeSpeed - 1000);

        navX = new NavXMicro(NavXMicro.NavXPort.I2C_onboard);
        navX.init();
        navX.zero();

        driveOdometry = new DifferentialDriveOdometry(getDriveHeading(), startingPose);

        resetPose();

        initDashboard();

        setDefaultCommand(new RunEndCommand(this::drive, diffDrive::stopMotor, this));
    }

    public void periodic() {
        updatePose();
    }

    public void initDashboard() {
        DashboardManager.addTab(this);
        dashboard_poseX = DashboardManager.addTabItem(this, "Pose/X", 0.0);
        dashboard_poseY = DashboardManager.addTabItem(this, "Pose/Y", 0.0);
        dashboard_poseHeadingDegrees = DashboardManager.addTabItem(this, "Pose/HeadingDeg", 0.0);
        dashboard_poseHeadingRadians = DashboardManager.addTabItem(this, "Pose/HeadingRad", 0.0);
        dahsboard_navXYaw = DashboardManager.addTabItem(this, "NavX Yaw", 0.0);
        dashboard_leftDistance = DashboardManager.addTabItem(this, "LeftDriveDistance", 0.0);
        dashboard_rightDistance = DashboardManager.addTabItem(this, "RightDriveDistance", 0.0);
        dashboard_leftOutputVolts = DashboardManager.addTabItem(this, "Left Volts", 0.0, DashboardWidget.Graph);
        dashboard_rightOutputVolts = DashboardManager.addTabItem(this, "Right Volts", 0.0, DashboardWidget.Graph);
        dashboard_leftWheelSpeedMPS = DashboardManager.addTabItem(this, "Left Wheel Speed", 0.0, DashboardWidget.Graph);
        dashboard_rightWheelSpeedMPS = DashboardManager.addTabItem(this, "Right Wheel Speed", 0.0, DashboardWidget.Graph);
        startPoseChooser = DashboardManager.addTabChooser(this, "StartPos", StartingPosition.values(), StartingPosition.kZeroZero);
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
        dashboard_rightWheelSpeedMPS.setDouble(-getRightVelocityMetersPerSec());
    }

    public void resetMCSettings () {
        leftMaster.wipeSettings();
        rightMaster.wipeSettings();
        leftSlave.wipeSettings();
        rightSlave.wipeSettings();
    }

    public void resetMCCurrentLimit () {
        rightMaster.limitInputCurrent(40);
        leftMaster.limitInputCurrent(40);
        rightSlave.limitInputCurrent(40);
        leftSlave.limitInputCurrent(40);
    }

    public void setMCCurrentLimit (int currentLimit) {
        rightMaster.limitInputCurrent(currentLimit);
        leftMaster.limitInputCurrent(currentLimit);
        rightSlave.limitInputCurrent(currentLimit);
        leftSlave.limitInputCurrent(currentLimit);
    }

    public boolean passedInit() {
        return initPassed;
    }

    private void updatePose() {
        pose = driveOdometry.update(getDriveHeading(), getLeftDistanceMeters(), getRightDistanceMeters());
    }

    public void updateDashboardPose() {
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

        dashboard_rightOutputVolts.setDouble(latestRightWheelVolts);
        dashboard_leftOutputVolts.setDouble(latestLeftWheelVolts);
    }

    public Rotation2d getDriveHeading() {
        return Rotation2d.fromDegrees(-navX.getYaw());
    }

    public double getRightDistanceMeters () {
        return Constants.Drivetrain.dtPowertrain.calculateWheelDistanceMeters(-rightMaster.getSensorPosition());
    }

    public double getLeftDistanceMeters () {
        return Constants.Drivetrain.dtPowertrain.calculateWheelDistanceMeters(leftMaster.getSensorPosition());
    }

    public double getRightVelocityMetersPerSec () {
        return Constants.Drivetrain.dtPowertrain.calculateMetersPerSec(rightMaster.getSensorVelocity());
    }

    public double getLeftVelocityMetersPerSec () {
        return Constants.Drivetrain.dtPowertrain.calculateMetersPerSec(leftMaster.getSensorVelocity());
    }

    public void resetPose() {
        resetPose(startingPose);
    }

    public void resetPoseFromChooser() {
        Pose2d startPose = startPoseChooser.getSelected().poseMeters;
        resetPose(startPose);
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

    public void stickDrive() {
        tankProfile.calculateTankSpeeds();
        diffDrive.tankDrive(tankProfile.leftPow, tankProfile.rightPow, tankProfile.loopMode);
    }

    public void xBoxDrive() {
        double rightPower = 0;
        double leftPower = 0;
        DriveAxesSupplier axes = oi.driverOI.getArcadeDriveAxes();
        rightPower = OscarMath.signumPow(-axes.getRight() * Constants.Drivetrain.XBoxDriveMultiplier, 3);
        leftPower = OscarMath.signumPow(axes.getLeft() * Constants.Drivetrain.XBoxDriveMultiplier, 3);
        diffDrive.arcadeDrive(rightPower, leftPower, SmartDiffDrive.LoopMode.PERCENTAGE);
    }

    public void drive() {
        if (oi.driverOI instanceof XboxDriverOI) {
            xBoxDrive();
        } else if (oi.driverOI instanceof SticksDriverOI) {
            stickDrive();
        }
    }

    public Pose2d getLatestPose() {
        updatePose();
        return pose;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds () {
        return new DifferentialDriveWheelSpeeds(getLeftVelocityMetersPerSec(), getRightVelocityMetersPerSec());
    }

    private double latestLeftWheelVolts, latestRightWheelVolts;

    public void setWheelVolts(Double leftVolts, Double rightVolts) {
        double leftBusVoltage = leftMaster.getInputVoltage();
        double rightBusVoltage = rightMaster.getInputVoltage();

        latestLeftWheelVolts = Math.abs(leftVolts / leftBusVoltage) * Math.signum(leftVolts);
        latestRightWheelVolts = -Math.abs(rightVolts / rightBusVoltage) * Math.signum(rightVolts);

        leftMaster.set(latestLeftWheelVolts);
        rightMaster.set(latestRightWheelVolts);
    }

    public void visionDrive(double distance, double yaw) {
        double moveSpeed = Constants.Drivetrain.visionMoveKp * distance;

        if (yaw <= Math.PI) OscarMath.map(yaw, 0, Math.PI, 0, 180);
        else OscarMath.map(yaw, Math.PI, 2*Math.PI, 0, -179);

        double rotSpeed = Constants.Drivetrain.visionRotKp * yaw;

        diffDrive.arcadeDrive(moveSpeed, rotSpeed, SmartDifferentialDrive.LoopMode.PERCENTAGE);
    }
}
