/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team832.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team832.lib.OscarTimedRobot;
import frc.team832.robot.subsystems.DrivetrainSubsystem;

public class Robot extends OscarTimedRobot {

  public static final OI oi = new OI();

  public static final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();

  @Override
  public void robotInit() {
    if (drivetrain.passedInit()) {
      System.out.println("INIT - DRIVETRAIN OK");
    } else {
      System.err.println("INIT - DRIVETRAIN FAIL");
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
  }
}
