package frc.team832.robot;

import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team832.lib.driverinput.controllers.Attack3;
import frc.team832.lib.driverinput.controllers.Extreme3DPro;
import frc.team832.lib.driverinput.controllers.StratComInterface;
import frc.team832.lib.driverinput.oi.*;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.robot.autonomous.AutonomousSequencer;
import frc.team832.robot.autonomous.SequenceOptions.*;
import frc.team832.robot.autonomous.Trajectories;
//import frc.team832.robot.commands.FollowPathCommand;


@SuppressWarnings("WeakerAccess")
public class OI {

    public final DriverOI driverOI;
    public static final boolean isSticks = true;
    public static final StratComInterface stratComInterface = new StratComInterface(isSticks ? 2 : 1);

    public final Attack3 leftStick;
    public final Extreme3DPro rightStick;

    public OI() {
        if (isSticks) {
            driverOI = new SticksDriverOI();
            leftStick = ((SticksDriverOI)driverOI).leftStick;
            rightStick = ((SticksDriverOI)driverOI).rightStick;
        } else {
            driverOI = new XboxDriverOI();
        }

        // do commands here

    }
}