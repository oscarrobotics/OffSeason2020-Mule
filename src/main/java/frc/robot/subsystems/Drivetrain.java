package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;

public class Drivetrain extends Subsystem {

    private SparkM leftMaster, leftSlave, rightMaster, rightSlave;


   public void initialize() {
      leftMaster = new Spark(Constants.leftMasterCANId);
      leftSlave = new Spark(Constants.leftSlaveCANId);
      rightMaster = new Spark(Constants.rightMasterCANId);
      rightSlave = new Spark(Constants.rightSlaveCANId);

   }


    @Override
    protected void initDefaultCommand() {}

}
