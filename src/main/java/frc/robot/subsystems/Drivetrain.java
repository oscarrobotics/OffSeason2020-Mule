package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.team832.lib.motorcontrol.vendor.CANSparkMax;

import static com.revrobotics.CANSparkMaxLowLevel.*;

public class Drivetrain extends SubsystemBase {
    private CANSparkMax leftMaster, leftSlave, rightMaster, rightSlave;

    public Drivetrain() {

    }

    public void initialize() {
        leftMaster = new CANSparkMax(Constants.leftMasterCANId, MotorType.kBrushless);
        leftSlave = new CANSparkMax(Constants.leftSlaveCANId, MotorType.kBrushless);
        rightMaster = new CANSparkMax(Constants.rightMasterCANId, MotorType.kBrushless);
        rightSlave = new CANSparkMax(Constants.rightSlaveCANId, MotorType.kBrushless);
   }
}
