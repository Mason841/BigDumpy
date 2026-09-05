package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;

public class Autoaim extends SubsystemBase {

    public double iteratedTOF;

    public Translation2d targetPose = new Translation2d();
    public Translation2d targetDisplacedIteratedPose = new Translation2d();

    public FiringLocation target = FiringLocation.HUB;

    private final Drivetrain drivetrain;

    private InterpolatingDoubleTreeMap hubTimeOfFlightMap;
    private InterpolatingDoubleTreeMap passTimeOfFlightMap;
	private InterpolatingDoubleTreeMap shooterSpeedsMap;
	private InterpolatingDoubleTreeMap passingShooterSpeedsMap;
    private InterpolatingDoubleTreeMap hoodHeightMap;

    public enum FiringLocation {
        HUB,
        PASS
    }

    public Autoaim(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        this.hubTimeOfFlightMap = new InterpolatingDoubleTreeMap();
		this.hubTimeOfFlightMap.put(1.0, HubTimeOfFlight.M1.getSeconds());
		this.hubTimeOfFlightMap.put(2.0, HubTimeOfFlight.M2.getSeconds());
		this.hubTimeOfFlightMap.put(3.0, HubTimeOfFlight.M3.getSeconds());
		this.hubTimeOfFlightMap.put(4.0, HubTimeOfFlight.M4.getSeconds());
		this.hubTimeOfFlightMap.put(5.0, HubTimeOfFlight.M5.getSeconds());
		this.hubTimeOfFlightMap.put(6.0, HubTimeOfFlight.M6.getSeconds());
		this.hubTimeOfFlightMap.put(7.0, HubTimeOfFlight.M7.getSeconds());

        this.passTimeOfFlightMap = new InterpolatingDoubleTreeMap();
		this.passTimeOfFlightMap.put(1.0, PassTimeOfFlight.M1.getSeconds());
		this.passTimeOfFlightMap.put(3.0, PassTimeOfFlight.M3.getSeconds());
		this.passTimeOfFlightMap.put(5.0, PassTimeOfFlight.M5.getSeconds());
		this.passTimeOfFlightMap.put(7.0, PassTimeOfFlight.M7.getSeconds());
        this.passTimeOfFlightMap.put(9.0, PassTimeOfFlight.M9.getSeconds());
		this.passTimeOfFlightMap.put(11.0, PassTimeOfFlight.M11.getSeconds());
		this.passTimeOfFlightMap.put(13.0, PassTimeOfFlight.M13.getSeconds());

        this.shooterSpeedsMap = new InterpolatingDoubleTreeMap();
		this.shooterSpeedsMap.put(1.0, ShooterSpeed.M1.getRPS());
		this.shooterSpeedsMap.put(2.0, ShooterSpeed.M2.getRPS());
		this.shooterSpeedsMap.put(3.0, ShooterSpeed.M3.getRPS());
		this.shooterSpeedsMap.put(4.0, ShooterSpeed.M4.getRPS());
		this.shooterSpeedsMap.put(5.0, ShooterSpeed.M5.getRPS());
		this.shooterSpeedsMap.put(6.0, ShooterSpeed.M6.getRPS());
		this.shooterSpeedsMap.put(7.0, ShooterSpeed.M7.getRPS());

		this.passingShooterSpeedsMap = new InterpolatingDoubleTreeMap();
		this.passingShooterSpeedsMap.put(0.0, PassingShooterSpeed.M0.getRPS());
		this.passingShooterSpeedsMap.put(4.0, PassingShooterSpeed.M4.getRPS());
		this.passingShooterSpeedsMap.put(8.0, PassingShooterSpeed.M8.getRPS());
		this.passingShooterSpeedsMap.put(10.0, PassingShooterSpeed.M10.getRPS());
		this.passingShooterSpeedsMap.put(12.0, PassingShooterSpeed.M12.getRPS());
		this.passingShooterSpeedsMap.put(16.0, PassingShooterSpeed.M16.getRPS());

        this.hoodHeightMap = new InterpolatingDoubleTreeMap();
        this.hoodHeightMap.put(1.0, HoodHeight.M1.getPosition());
        this.hoodHeightMap.put(5.0, HoodHeight.M5.getPosition());
        this.hoodHeightMap.put(10.0, HoodHeight.M10.getPosition());

    }

    public void setFiringLocation(FiringLocation newLocation) {
        target = newLocation;
    }

    public double getShooterSpeedToTargetWhileMoving() {
        switch (target) {
            case HUB:
                return shooterSpeedsMap.get(this.getDistanceToScoreWhileMoving());

            case PASS:
                return passingShooterSpeedsMap.get(this.getDistanceToScoreWhileMoving());
        
            default:
                return 0;
        }
    }

    public double getHoodPositionToTargetWhileMoving() {
        switch (target) {
            case HUB:
                return hoodHeightMap.get(this.getDistanceToScoreWhileMoving());

            case PASS:
                return 20;
        
            default:
                return 0;
        }
    }

    public Rotation2d getFieldRelativeAngleToFireWhileMoving() {
        return new Rotation2d(
            Math.atan2(
                targetDisplacedIteratedPose.getY() - drivetrain.getState().Pose.getY(), 
                targetDisplacedIteratedPose.getX() - drivetrain.getState().Pose.getX()));
    }

    public double getDistanceToScoreWhileMoving() {
        return drivetrain.getState().Pose.getTranslation().getDistance(targetDisplacedIteratedPose);
    }

    private double getDistanceToTargetWhileMoving(double ballTOF) {
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                drivetrain.getState().Speeds, drivetrain.getState().Pose.getRotation());

        Translation2d velocityDisplacementOverTime = new Translation2d(
            fieldRelativeSpeeds.vxMetersPerSecond * ballTOF, fieldRelativeSpeeds.vyMetersPerSecond * ballTOF);

        return targetPose.minus(velocityDisplacementOverTime).getDistance(drivetrain.getState().Pose.getTranslation());
    }

    private Translation2d getDisplacedTarget(double ballTOF) {
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                drivetrain.getState().Speeds, drivetrain.getState().Pose.getRotation());

        Translation2d velocityDisplacementOverTime = new Translation2d(
            fieldRelativeSpeeds.vxMetersPerSecond * ballTOF, fieldRelativeSpeeds.vyMetersPerSecond * ballTOF);

        return targetPose.minus(velocityDisplacementOverTime);
    }

    private double getDistanceToTarget() {
        return targetPose.getDistance(drivetrain.getState().Pose.getTranslation());
    }

    public boolean goodToPass() {
        if (drivetrain.getState().Pose.getY() > RobotConstants.AutoAimConstants.HUB_BLOCKS_PASSING_Y_TOP || 
            drivetrain.getState().Pose.getY() < RobotConstants.AutoAimConstants.HUB_BLOCKS_PASSING_Y_BOTTOM) {
            return true;
        }
        if (RobotConstants.isRedAlliance.getAsBoolean()) {
            if (drivetrain.getState().Pose.getX() < RobotConstants.AutoAimConstants.FIELD_LENGTH_X - RobotConstants.AutoAimConstants.MINIMUM_TO_PASS_BEHIND_HUB_X) {
                return true;
            }
        } else {
            if (drivetrain.getState().Pose.getX() > RobotConstants.AutoAimConstants.MINIMUM_TO_PASS_BEHIND_HUB_X) {
                return true;
            }
        }
        return false;
    }

    @Override
    public void periodic() {
        switch (target) {
            case HUB:
                if (RobotConstants.isRedAlliance.getAsBoolean()) {
                    targetPose = RobotConstants.AutoAimConstants.RED_HUB_POSE.getTranslation();
                } else {
                    targetPose = RobotConstants.AutoAimConstants.BLUE_HUB_POSE.getTranslation();
                }
                break;

            case PASS:
                if (RobotConstants.isRedAlliance.getAsBoolean()) {
                    if (drivetrain.getState().Pose.getY() > (RobotConstants.AutoAimConstants.FIELD_WIDTH_Y / 2)) {
                        if (drivetrain.getState().Pose.getY() > RobotConstants.AutoAimConstants.PASSING_SWAP_TARGET_POSE_Y_TOP) {
                            targetPose = RobotConstants.AutoAimConstants.RED_PASS_SHOT_HIGH_POSE.getTranslation();
                        } else {
                            targetPose = RobotConstants.AutoAimConstants.RED_PASS_SHOT_HIGH_POSE_BEHIND_HUB.getTranslation();
                        }
                    } else {
                        if (drivetrain.getState().Pose.getY() < RobotConstants.AutoAimConstants.PASSING_SWAP_TARGET_Y_BOTTOM) {
                            targetPose = RobotConstants.AutoAimConstants.RED_PASS_SHOT_LOW_POSE.getTranslation();
                        } else {
                            targetPose = RobotConstants.AutoAimConstants.RED_PASS_SHOT_LOW_POSE_BEHIND_HUB.getTranslation();
                        }
                    }
                } else {
                    if (drivetrain.getState().Pose.getY() > (RobotConstants.AutoAimConstants.FIELD_WIDTH_Y / 2)) {
                        if (drivetrain.getState().Pose.getY() > RobotConstants.AutoAimConstants.PASSING_SWAP_TARGET_POSE_Y_TOP) {
                            targetPose = RobotConstants.AutoAimConstants.BLUE_PASS_SHOT_HIGH_POSE.getTranslation();
                        } else {
                            targetPose = RobotConstants.AutoAimConstants.BLUE_PASS_SHOT_HIGH_POSE_BEHIND_HUB.getTranslation();
                        }                    
                    } else {
                        if (drivetrain.getState().Pose.getY() < RobotConstants.AutoAimConstants.PASSING_SWAP_TARGET_Y_BOTTOM) {
                            targetPose = RobotConstants.AutoAimConstants.BLUE_PASS_SHOT_LOW_POSE.getTranslation();
                        } else {
                            targetPose = RobotConstants.AutoAimConstants.BLUE_PASS_SHOT_LOW_POSE_BEHIND_HUB.getTranslation();
                        }                   
                    }
                }

            default:
                break;
        }

        iteratedTOF();
        
        try {
            Logger.recordOutput("Autoaim/target", new Pose2d(targetPose, new Rotation2d()));
            Logger.recordOutput("Autoaim/displacedTarget", new Pose2d(targetDisplacedIteratedPose, new Rotation2d()));
            Logger.recordOutput("Autoaim/iteratedTOF", iteratedTOF);
            Logger.recordOutput("Autoaim/drivetrainAngle", drivetrain.getState().Pose.getRotation());
            Logger.recordOutput("Autoaim/distanceToTarget", this.getDistanceToTarget());
            Logger.recordOutput("Autoaim/distanceToDisplacedTarget", this.getDistanceToScoreWhileMoving());
            Logger.recordOutput("Autoaim/goodToPass", this.goodToPass());
        } catch (Exception e) {
            System.out.println("Error logging autoaim values: " + e.getMessage());
        }
    }

    private void iteratedTOF() {
        switch (target) {
            case HUB:
                double hubCycleTOF = hubTimeOfFlightMap.get(getDistanceToTarget());
                for (int i = 0; i < 5; i++) {
                    hubCycleTOF = hubTimeOfFlightMap.get(getDistanceToTargetWhileMoving(hubCycleTOF));
                }
                iteratedTOF = hubCycleTOF;
                targetDisplacedIteratedPose = getDisplacedTarget(iteratedTOF);
                break;

            case PASS:
                double passCycleTOF = passTimeOfFlightMap.get(getDistanceToTarget());
                for (int i = 0; i < 5; i++) {
                    passCycleTOF = passTimeOfFlightMap.get(getDistanceToTargetWhileMoving(passCycleTOF));
                }
                iteratedTOF = passCycleTOF;
                targetDisplacedIteratedPose = getDisplacedTarget(iteratedTOF);
                break;
        
            default:
                break;
        }
    }

    private enum HubTimeOfFlight {

		M1(0.7),
		M2(0.9),
		M3(1.1),
		M4(1.3),
		M5(1.5),
		M6(1.7),
		M7(1.9);

		private final double sec;

		HubTimeOfFlight(double sec) {
			this.sec = sec;
		}

		public double getSeconds() {
			return sec;
		}
	}

    private enum PassTimeOfFlight {

		M1(0.2),
		M3(0.4),
		M5(0.6),
		M7(0.8),
		M9(1),
		M11(1.2),
		M13(1.4);

		private final double sec;

		PassTimeOfFlight(double sec) {
			this.sec = sec;
		}

		public double getSeconds() {
			return sec;
		}
	}

    public enum ShooterSpeed {

		M1(-36),
		M2(-41.5),
		M3(-46.5),
		M4(-51),
		M5(-54),
		M6(-58),
		M7(-61);

		private final double rps;

		ShooterSpeed(double rps) {
			this.rps = rps;
		}

		public double getRPS() {
			return rps;
		}
	}

	public enum PassingShooterSpeed {

		M0(-15),
		M2(-33),
		M4(-44),
		M6(-56),
		M8(-65),
		M10(-73),
		M12(-86),
		M14(-99),
		M16(-100);

		private final double rps;

		PassingShooterSpeed(double rps) {
			this.rps = rps;
		}

		public double getRPS() {
			return rps;
		}
	}
    
    public enum HoodHeight {

		M1(0),
		M5(-2),
        M10(-4.1);

		private final double position;

		HoodHeight(double position) {
			this.position = position;
		}

		public double getPosition() {
			return position;
		}
	}
}
