/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team832.robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team832.lib.OscarTimedRobot;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.robot.subsystems.DrivetrainSubsystem;

public class Robot extends OscarTimedRobot {

  public static final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  public static final OI oi = new OI();

  private static final Notifier drivetrainTelemetryNotifier = new Notifier(drivetrain::updateDashboardData);

  public static final SendableChooser<StartingPosition> startPoseChooser = new SendableChooser<>();

  @Override
  public void robotInit() {
//    startPoseChooser.setDefaultOption("ZeroZero", StartingPosition.kZeroZero);
//    ShuffleboardTab dtTab = DashboardManager.getTab(drivetrain);
//    dtTab.add(startPoseChooser);
//    dtTab.add("Reset Pose", new InstantCommand(drivetrain::resetPoseFromChooser));

    if (drivetrain.passedInit()) {
      System.out.println("INIT - DRIVETRAIN OK");
      drivetrainTelemetryNotifier.startPeriodic(0.05);
    } else {
      System.err.println("INIT - DRIVETRAIN FAIL");
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    drivetrain.updateDashboardData();
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
