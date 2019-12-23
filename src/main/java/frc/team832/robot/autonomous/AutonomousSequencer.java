package frc.team832.robot.autonomous;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team832.robot.Constants;
import frc.team832.robot.Robot;
import frc.team832.robot.autonomous.SequenceOptions.*;
import frc.team832.robot.commands.examples.EjectCargo;
import frc.team832.robot.commands.examples.EjectHatch;
import frc.team832.robot.commands.FollowPathCommand;
import frc.team832.robot.commands.examples.ObtainCargo;
import frc.team832.robot.commands.examples.ObtainHatch;

import java.util.List;


public class AutonomousSequencer {
    public final StartingPosition startPosition;
    public final PrimaryPath primaryPath;
    public final AutoTask primaryTask;
    public final SecondaryPath secondaryPath;
    public final AutoTask secondaryTask;
    public final TertiaryPath tertiaryPath;
    public final AutoTask tertiaryTask;
    public final boolean preGenerated;

    public AutonomousSequencer(StartingPosition startPosition, PrimaryPath primaryPath, AutoTask primaryTask, SecondaryPath secondaryPath, AutoTask secondaryTask, TertiaryPath tertiaryPath, AutoTask tertiaryTask, boolean isPreGenerated) {
        this(startPosition, primaryPath, primaryTask, secondaryPath, secondaryTask, tertiaryPath, tertiaryTask, 0, isPreGenerated);
    }

    public AutonomousSequencer(StartingPosition startPosition, PrimaryPath primaryPath, AutoTask primaryTask, SecondaryPath secondaryPath, AutoTask secondaryTask, TertiaryPath tertiaryPath, AutoTask tertiaryTask, int startDelayMs, boolean isPreGenerated) {
        this.startPosition = startPosition;
        this.primaryPath = primaryPath;
        this.primaryTask = primaryTask;
        this.secondaryPath = secondaryPath;
        this.secondaryTask = secondaryTask;
        this.tertiaryPath = tertiaryPath;
        this.tertiaryTask = tertiaryTask;
        this.preGenerated = isPreGenerated;
    }

    private static final Command doNothingCommand = new PrintCommand("DoNothing");

    public SequentialCommandGroup composeCommandGroup() {
        Command primaryCommand;
        Command secondaryCommand;
        Command tertiaryCommand;

        switch (primaryTask) {
            case EJECT_HATCH:
                primaryCommand = new EjectHatch();
                break;
            case EJECT_CARGO:
                primaryCommand = new EjectCargo();
                break;
            default:
                primaryCommand = doNothingCommand;
        }

        switch (secondaryTask) {
            case OBTAIN_HATCH:
                secondaryCommand = new ObtainHatch();
                break;
            case OBTAIN_CARGO:
                secondaryCommand = new ObtainCargo();
                break;
            default:
                secondaryCommand = doNothingCommand;
        }

        switch (tertiaryTask) {
            case EJECT_HATCH:
                tertiaryCommand = new EjectHatch();
                break;
            case EJECT_CARGO:
                tertiaryCommand = new EjectCargo();
                break;
            default:
                tertiaryCommand = doNothingCommand;
        }

        if (!preGenerated) {

            var priTraj = TrajectoryGenerator.generateTrajectory(List.of(startPosition.poseMeters, primaryPath.endPos.poseMeters), Constants.Drivetrain.kMedTrajConfig);
            var secTraj = TrajectoryGenerator.generateTrajectory(Robot.drivetrain.pose, List.of(new Translation2d(3.5, 8.2 - 6)), secondaryPath.endPos.poseMeters, Constants.Drivetrain.kMedTrajConfig);
            var terTraj = TrajectoryGenerator.generateTrajectory( List.of(Robot.drivetrain.pose, new Pose2d(5.1, 1.8, Rotation2d.fromDegrees(200)),tertiaryPath.endPos.poseMeters), Constants.Drivetrain.kSlowTrajConfig);

            System.out.println(priTraj.toString());

            return new SequentialCommandGroup(
                    new FollowPathCommand(priTraj), primaryCommand, new FollowPathCommand(secTraj), secondaryCommand, new FollowPathCommand(terTraj), tertiaryCommand);
//                    new FollowPathCommand(TrajectoryGenerator.generateTrajectory(primaryPath.endPos.poseMeters, List.of(), secondaryPath.endPos.poseMeters, Constants.Drivetrain.kTrajectoryConfig)),
//                    secondaryCommand,
//                    new FollowPathCommand(TrajectoryGenerator.generateTrajectory(secondaryPath.endPos.poseMeters, List.of(), tertiaryPath.endPos.poseMeters, Constants.Drivetrain.kTrajectoryConfig)),
//                    tertiaryCommand);
        } else {
            return new SequentialCommandGroup(
                    new FollowPathCommand(primaryPath.getPath(startPosition)),
                    primaryCommand,
                    new FollowPathCommand(secondaryPath.getPath(primaryPath.endPos)),
                    secondaryCommand,
                    new FollowPathCommand(tertiaryPath.getPath(secondaryPath.endPos)),
                    tertiaryCommand);
        }
    }
}
