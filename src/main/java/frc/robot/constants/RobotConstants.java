package frc.robot.constants;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class RobotConstants {

    public static BooleanSupplier isRedAlliance = () -> {
        var alliance = DriverStation.getAlliance();
        return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
    };

    public static BooleanSupplier isAuto = () -> {
        return DriverStation.isAutonomousEnabled();
    };

    public static class Vision {

        public static final String limelightName = "limelight-front";

        public static final double[] limelightPose = {0.3406, 0.0, 0.3697, 0.0, 20.0, 0.0};
    }

    public static class AutoAimConstants {

        // All measurements in meters

        public static final double FIELD_LENGTH_X = 16.54175;
        public static final double FIELD_WIDTH_Y = 8.06958;
        public static final double MINIMUM_TO_PASS_BEHIND_HUB_X = 6.75;

        public static final double HUB_BLOCKS_PASSING_Y_TOP = 4.7;
        public static final double HUB_BLOCKS_PASSING_Y_BOTTOM = 3.3;

        public static final double PASSING_SWAP_TARGET_POSE_Y_TOP = 5;
        public static final double PASSING_SWAP_TARGET_Y_BOTTOM = 3;
        
        public static final Pose2d BLUE_HUB_POSE = new Pose2d(4.626, 4.035, Rotation2d.kZero);
        public static final Pose2d RED_HUB_POSE = new Pose2d(11.914, 4.035, Rotation2d.kZero);

        public static final Pose2d BLUE_PASS_SHOT_HIGH_POSE = new Pose2d(3, 5.4, Rotation2d.kZero);
        public static final Pose2d BLUE_PASS_SHOT_LOW_POSE = new Pose2d(3, 2, Rotation2d.kZero);

        public static final Pose2d BLUE_PASS_SHOT_HIGH_POSE_BEHIND_HUB = new Pose2d(3, 6.4, Rotation2d.kZero);
        public static final Pose2d BLUE_PASS_SHOT_LOW_POSE_BEHIND_HUB = new Pose2d(3, 1.7, Rotation2d.kZero);

        public static final Pose2d RED_PASS_SHOT_HIGH_POSE = new Pose2d(13.54175, 5.4, Rotation2d.kZero);
        public static final Pose2d RED_PASS_SHOT_LOW_POSE = new Pose2d(13.54175, 2.6, Rotation2d.kZero);
        
        public static final Pose2d RED_PASS_SHOT_HIGH_POSE_BEHIND_HUB = new Pose2d(13.54175, 6.4, Rotation2d.kZero);
        public static final Pose2d RED_PASS_SHOT_LOW_POSE_BEHIND_HUB = new Pose2d(13.54175, 1.7, Rotation2d.kZero);

        public static final Pose2d OUTPOST_BLUE = new Pose2d(2.885, 0.933, Rotation2d.kZero);
        public static final Pose2d OUTPUT_RED = new Pose2d(14.392, 7.379, Rotation2d.kZero);
    }
}
