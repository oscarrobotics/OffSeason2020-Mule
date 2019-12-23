package frc.team832.robot;

import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team832.lib.driverinput.controllers.Attack3;
import frc.team832.lib.driverinput.controllers.Extreme3DPro;
import frc.team832.lib.driverinput.oi.*;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.robot.autonomous.AutonomousSequencer;
import frc.team832.robot.autonomous.SequenceOptions.*;
import frc.team832.robot.autonomous.Trajectories;
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
        InstantCommand resetPoseFromChooser = new InstantCommand(Robot.drivetrain::resetPoseFromChooser, Robot.drivetrain);
        DashboardManager.getTab(Robot.drivetrain).add("ResetPoseFromChooser", resetPoseFromChooser);
        leftStick.eleven.whenPressed(resetPoseFromChooser);
        leftStick.two.whenPressed(Robot.drivetrain::resetPose, Robot.drivetrain);
        leftStick.three.whenPressed(new FollowPathCommand(Trajectories.test));
        leftStick.eight.whenPressed(new FollowPathCommand(Trajectories.example));

        CommandGroupBase testAuto = new AutonomousSequencer(StartingPosition.kHAB1Right,
                new PrimaryPath(PrimaryDestination.CARGO_FRONT_RIGHT), AutoTask.EJECT_HATCH,
                new SecondaryPath(SecondaryDestination.RIGHT_HP_HATCH), AutoTask.OBTAIN_HATCH,
                new TertiaryPath(TertiaryDestination.RIGHT_CARGO_SIDE2), AutoTask.DO_NOTHING, false)
                .composeCommandGroup();

        leftStick.six.whenPressed(testAuto);
    }
}