package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.RunEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.drive.SmartDifferentialDrive;
import frc.team832.lib.driverinput.oi.DriveAxesSupplier;
import frc.team832.lib.driverinput.oi.SticksDriverOI;
import frc.team832.lib.driverinput.oi.XboxDriverOI;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol.vendor.CANSparkMax;
import frc.team832.lib.motors.Motors;
import frc.team832.lib.sensors.NavXMicro;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;
import frc.team832.robot.Robot;

import static com.revrobotics.CANSparkMaxLowLevel.*;
import static frc.team832.robot.Robot.oi;

@SuppressWarnings({"FieldCanBeLocal", "WeakerAccess"})
public class DrivetrainSubsystem extends SubsystemBase {
    private final CANSparkMax leftMaster, leftSlave, rightMaster, rightSlave;

    public ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    public DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds();

    private final SmartDifferentialDrive diffDrive;
    public Pose2d pose = new Pose2d();
//    public DifferentialDriveOdometry driveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(Robot.drivetrain.navX.getYaw()) , Constants.startPose);

    public NavXMicro navX;

    private boolean initPassed;

    public double leftVelocity;
    public double rightVelocity;

    public DrivetrainSubsystem() {
        leftMaster = new CANSparkMax(Constants.leftMasterCANId, MotorType.kBrushless);
        leftSlave = new CANSparkMax(Constants.leftSlaveCANId, MotorType.kBrushless);
        rightMaster = new CANSparkMax(Constants.rightMasterCANId, MotorType.kBrushless);
        rightSlave = new CANSparkMax(Constants.rightSlaveCANId, MotorType.kBrushless);

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

//        navX = new NavXMicro(NavXMicro.NavXPort.I2C_onboard);
//        navX.zero();

        setDefaultCommand(new RunEndCommand(this::drive, diffDrive::stopMotor, this));
    }

    public void periodic(){

//        Rotation2d gyroAngle = Rotation2d.fromDegrees(-navX.getYaw());

//        pose = driveOdometry.update(gyroAngle, Constants.dtPowertrain.calculateWheelDistanceMeters(leftMaster.getSensorPosition()), Constants.dtPowertrain.calculateWheelDistanceMeters(rightMaster.getSensorPosition()));
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
}
