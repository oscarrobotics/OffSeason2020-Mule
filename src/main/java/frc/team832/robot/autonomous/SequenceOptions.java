package frc.team832.robot.autonomous;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.team832.robot.Constants;

public class SequenceOptions {

    public enum StartingPosition {
        kZeroZero(Constants.Poses.kZeroZeroPose),
        kHAB1Left(Constants.Poses.kLeftHABStartPose),
        kHAB1Center(Constants.Poses.kCenterHABStartPose),
        kHAB1Right(Constants.Poses.kRightHABStartPose);

        public final Pose2d poseMeters;

        StartingPosition (Pose2d pose) {
            poseMeters = pose;
        }
    }

    public enum AutoTask {
        EJECT_HATCH,
        EJECT_CARGO,
        OBTAIN_HATCH,
        OBTAIN_CARGO,
        DO_NOTHING
    }

    public class PrimaryPath {

        public final PrimaryDestination endPos;

        public PrimaryPath(PrimaryDestination end) {
            this.endPos = end;
        }

        Trajectory getPath(StartingPosition startPos) {
            switch (startPos) {
                case kHAB1Left:
                    switch (endPos) {
                        case ROCKET_CLOSE:
                        case ROCKET_FAR:
                        case CARGO_LEFTSIDE1:
                        case CARGO_LEFTSIDE2:
                        case CARGO_LEFTSIDE3:
                        default:
                            return null;
                    }
                case kHAB1Center:
                    switch (endPos) {
                        case CARGO_FRONT_LEFT:
                        case CARGO_FRONT_RIGHT:
                        default:
                            return null;
                    }
                case kHAB1Right:
                    switch (endPos) {
                        case ROCKET_CLOSE:
                        case ROCKET_FAR:
                        case CARGO_RIGHTSIDE1:
                        case CARGO_RIGHTSIDE2:
                        case CARGO_RIGHTSIDE3:
                        default:
                            return null;
                    }
            }
            return null;
        }
    }

    public enum PrimaryDestination {
        ROCKET_CLOSE(new Pose2d()),
        ROCKET_FAR(new Pose2d()),
        CARGO_FRONT_LEFT(new Pose2d()),
        CARGO_FRONT_RIGHT(new Pose2d()),
        CARGO_LEFTSIDE1(new Pose2d()),
        CARGO_LEFTSIDE2(new Pose2d()),
        CARGO_LEFTSIDE3(new Pose2d()),
        CARGO_RIGHTSIDE1(new Pose2d()),
        CARGO_RIGHTSIDE2(new Pose2d()),
        CARGO_RIGHTSIDE3(new Pose2d());

        public final Pose2d poseMeters;

        PrimaryDestination(Pose2d pose) {
            poseMeters = pose;
        }
    }

    public class SecondaryPath {

        public final SecondaryDestination endPos;

        public SecondaryPath(SecondaryDestination end) {
            this.endPos = end;
        }

        Trajectory getPath(PrimaryDestination startPos) {
            switch (startPos) {
                case ROCKET_CLOSE:
                    switch (endPos) {
                        case HP_HATCH:
                        case CARGO_PIT:
                        case DO_NOTHING:
                        default:
                            return null;
                    }
                case ROCKET_FAR:
                    switch (endPos) {
                        case HP_HATCH:
                        case CARGO_PIT:
                        case DO_NOTHING:
                        default:
                            return null;
                    }
                case CARGO_FRONT_LEFT:
                    switch (endPos) {
                        case HP_HATCH:
                        case CARGO_PIT:
                        case DO_NOTHING:
                        default:
                            return null;
                    }
                case CARGO_FRONT_RIGHT:
                    switch (endPos) {
                        case HP_HATCH:
                        case CARGO_PIT:
                        case DO_NOTHING:
                        default:
                            return null;
                    }
                case CARGO_LEFTSIDE1:
                    switch (endPos) {
                        case HP_HATCH:
                        case CARGO_PIT:
                        case DO_NOTHING:
                        default:
                            return null;
                    }
                case CARGO_LEFTSIDE2:
                    switch (endPos) {
                        case HP_HATCH:
                        case CARGO_PIT:
                        case DO_NOTHING:
                        default:
                            return null;
                    }
                case CARGO_LEFTSIDE3:
                    switch (endPos) {
                        case HP_HATCH:
                        case CARGO_PIT:
                        case DO_NOTHING:
                        default:
                            return null;
                    }
                case CARGO_RIGHTSIDE1:
                    switch (endPos) {
                        case HP_HATCH:
                        case CARGO_PIT:
                        case DO_NOTHING:
                        default:
                            return null;
                    }
                case CARGO_RIGHTSIDE2:
                    switch (endPos) {
                        case HP_HATCH:
                        case CARGO_PIT:
                        case DO_NOTHING:
                        default:
                            return null;
                    }
                case CARGO_RIGHTSIDE3:
                    switch (endPos) {
                        case HP_HATCH:
                        case CARGO_PIT:
                        case DO_NOTHING:
                        default:
                            return null;
                    }
            }
            return null;
        }
    }

    public enum SecondaryDestination {
        DO_NOTHING(new Pose2d()),
        CARGO_PIT(new Pose2d()),
        HP_HATCH(new Pose2d());

        public final Pose2d poseMeters;

        SecondaryDestination(Pose2d pose) {
            poseMeters = pose;
        }
    }

    public class TertiaryPath {

        public final TertiaryDestination endPos;

        public TertiaryPath(TertiaryDestination end) {
            this.endPos = end;
        }

        Trajectory getPath(SecondaryDestination startPos) {
            switch (startPos) {
                case HP_HATCH:
                    switch (endPos) {
                        case ROCKET_FAR:
                        case ROCKET_CLOSE:
                        case CARGO_SIDE1:
                        case CARGO_SIDE2:
                        case CARGO_SIDE3:
                        case DO_NOTHING:
                        default:
                            return null;
                    }
                case CARGO_PIT:
                    switch (endPos) {
                        case ROCKET_FAR:
                        case ROCKET_CLOSE:
                        case CARGO_SIDE1:
                        case CARGO_SIDE2:
                        case CARGO_SIDE3:
                        case DO_NOTHING:
                        default:
                            return null;
                    }
                case DO_NOTHING:
                    switch (endPos) {
                        case ROCKET_FAR:
                        case ROCKET_CLOSE:
                        case CARGO_SIDE1:
                        case CARGO_SIDE2:
                        case CARGO_SIDE3:
                        case DO_NOTHING:
                        default:
                            return null;
                    }
            }
            return null;
        }
    }

    public enum TertiaryDestination {
        DO_NOTHING(new Pose2d()),
        ROCKET_FAR(new Pose2d()),
        ROCKET_CLOSE(new Pose2d()),
        CARGO_SIDE1(new Pose2d()),
        CARGO_SIDE2(new Pose2d()),
        CARGO_SIDE3(new Pose2d());

        public final Pose2d poseMeters;

        TertiaryDestination(Pose2d pose) {
            poseMeters = pose;
        }

    }


}
