/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team832.robot;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team832.lib.OscarTimedRobot;
import frc.team832.robot.subsystems.DrivetrainSubsystem;
import frc.team832.robot.subsystems.ShooterSubsystem;

public class Robot extends OscarTimedRobot {

//  public static final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  public static final ShooterSubsystem shooter = new ShooterSubsystem();
  public static final OI oi = new OI();
  public static final PigeonIMU pigeon = new PigeonIMU(1);

  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);


  //  private static final Notifier drivetrainTelemetryNotifier = new Notifier(drivetrain::updateDashboardData);
  private static final Notifier shooterTelemetryNotifier = new Notifier(shooter::updateDashboardData);

  @Override
  public void robotInit() {
//    if (drivetrain.passedInit()) {
//      System.out.println("INIT - DRIVETRAIN OK");
//      drivetrainTelemetryNotifier.startPeriodic(0.05);
//    } else {
//      System.err.println("INIT - DRIVETRAIN FAIL");
//    }
    if (shooter.passedInit()) {
      System.out.println("INIT - SHOOTER OK");
      shooterTelemetryNotifier.startPeriodic(0.05);
    } else {
      System.err.println("INIT - SHOOTER FAIL");
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

//    Color detectedColor = m_colorSensor.getColor();
//
//    double IR = m_colorSensor.getIR();
//
//    SmartDashboard.putNumber("Red", detectedColor.red);
//    SmartDashboard.putNumber("Green", detectedColor.green);
//    SmartDashboard.putNumber("Blue", detectedColor.blue);
//    SmartDashboard.putNumber("IR", IR);
//
//    int proximity = m_colorSensor.getProximity();
//
//    SmartDashboard.putNumber("Proximity", proximity);
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
