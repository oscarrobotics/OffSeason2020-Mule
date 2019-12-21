package frc.team832.robot;

import frc.team832.lib.driverinput.controllers.Attack3;
import frc.team832.lib.driverinput.controllers.Extreme3DPro;
import frc.team832.lib.driverinput.oi.*;
import frc.team832.robot.commands.FollowPathCommand;

@SuppressWarnings("WeakerAccess")
public class OI {

    public final DriverOI driverOI;

    public final Attack3 leftStick;
    public final Extreme3DPro rightStick;

    public OI() {
//        driverOI = new XboxDriverOI();
         driverOI = new SticksDriverOI();

        leftStick = ((SticksDriverOI)driverOI).leftStick;
        rightStick = ((SticksDriverOI)driverOI).rightStick;

        // do commands here
        leftStick.two.whenPressed(Robot.drivetrain::resetPose, Robot.drivetrain);
        leftStick.three.whenPressed(new FollowPathCommand(Trajectories.test));
        leftStick.eight.whenPressed(new FollowPathCommand(Trajectories.example));
    }
}