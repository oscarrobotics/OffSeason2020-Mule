package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj2.command.RunEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.drive.SmartDifferentialDrive;
import frc.team832.lib.motorcontrol.vendor.CANSparkMax;
import frc.team832.lib.motors.Motors;
import frc.team832.robot.Constants;
import frc.team832.robot.Robot;

import static com.revrobotics.CANSparkMaxLowLevel.*;

@SuppressWarnings({"FieldCanBeLocal", "WeakerAccess"})
public class DrivetrainSubsystem extends SubsystemBase {
    private final CANSparkMax leftMaster, leftSlave, rightMaster, rightSlave;

    private final SmartDifferentialDrive diffDrive;

    private boolean initPassed;

    public DrivetrainSubsystem() {
        leftMaster = new CANSparkMax(Constants.leftMasterCANId, MotorType.kBrushless);
        leftSlave = new CANSparkMax(Constants.leftSlaveCANId, MotorType.kBrushless);
        rightMaster = new CANSparkMax(Constants.rightMasterCANId, MotorType.kBrushless);
        rightSlave = new CANSparkMax(Constants.rightSlaveCANId, MotorType.kBrushless);

        if (!(leftMaster.getInputVoltage() > 0)) initPassed = false;
        if (!(leftSlave.getInputVoltage() > 0)) initPassed = false;
        if (!(rightMaster.getInputVoltage() > 0)) initPassed = false;
        if (!(rightSlave.getInputVoltage() > 0)) initPassed = false;

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        rightMaster.setInverted(true);

        diffDrive = new SmartDifferentialDrive(leftMaster, rightMaster, Motors.NEO.freeSpeed - 1000);

        setDefaultCommand(new RunEndCommand(this::drive, diffDrive::stopMotor, this));
    }

    public boolean passedInit() {
        return initPassed;
    }

    public void drive() {
        var axes = Robot.oi.driverOI.getTankDriveAxes();
        diffDrive.tankDrive(axes.getLeft(), axes.getRight(), SmartDifferentialDrive.LoopMode.PERCENTAGE);
    }
}
