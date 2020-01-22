/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team832.robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.AddressableLED;
import edu.wpi.first.wpilibj2.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.AddressableLEDColorOrder;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team832.lib.OscarTimedRobot;
import frc.team832.robot.subsystems.DrivetrainSubsystem;

public class Robot extends OscarTimedRobot {

	public static final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
	public static final OI oi = new OI();

	public static final AddressableLED leds = new AddressableLED(0);
	public static final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(100, AddressableLEDColorOrder.kBRG);

	private static final Notifier drivetrainTelemetryNotifier = new Notifier(drivetrain::updateDashboardData);

	@Override
	public void robotInit () {
		if (drivetrain.passedInit()) {
			System.out.println("INIT - DRIVETRAIN OK");
			drivetrainTelemetryNotifier.startPeriodic(0.05);
		} else {
			System.err.println("INIT - DRIVETRAIN FAIL");
		}

		for (int i = 0; i < ledBuffer.getLength(); i++) {
			ledBuffer.setRGB(i, 0, 255, 0);
		}

		leds.setLength(100);
		leds.start();
		leds.setData(ledBuffer);

	}

	@Override
	public void robotPeriodic () {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void autonomousInit () {
	}

	@Override
	public void autonomousPeriodic () {
	}

	@Override
	public void teleopInit () {
	}

	@Override
	public void teleopPeriodic () {
	}
}
