// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.SuperstructureConstants;
import frc.robot.subsystems.Autoaim.FiringLocation;
import frc.robot.subsystems.Drivetrain.DriveState;
import frc.robot.subsystems.Hood.HoodState;
import frc.robot.subsystems.Indexer.IndexerState;
import frc.robot.subsystems.IntakeRack.IntakeRackState;
import frc.robot.subsystems.IntakeRoller.IntakeRollerState;
import frc.robot.subsystems.RollerFloor.RollerFloorState;
import frc.robot.subsystems.Shooter.ShooterState;

public class SubsystemManager extends SubsystemBase {

	private final Autoaim autoaim;

	private final Drivetrain drivetrain;
	private final Hood hood;
	private final Indexer indexer;
	private final IntakeRack intakeRack;
	private final IntakeRoller intakeRoller;
	private final RollerFloor rollerFloor;
	private final Shooter shooter;

	public enum RobotState {
		STOPPED,
		IDLE,
		DRIVING_STOWED,
		DRIVING_IDLE,
		DRIVING_INTAKING,
		DRIVING_INTAKEDEPLOYED,
		DRIVING_REVERSEINTAKING,
		DRIVING_SCORING,
		DRIVING_SCORINGMOVING
	}

	private RobotState state = RobotState.DRIVING_STOWED;

	private Timer shootTimer = new Timer();

	public SubsystemManager(Autoaim autoaim,
			Drivetrain drivetrain, Hood hood, Indexer indexer, IntakeRack intakeRack, IntakeRoller intakeRoller,
			RollerFloor rollerFloor, Shooter shooter) {

		this.autoaim = autoaim;

		this.drivetrain = drivetrain;
		this.hood = hood;
		this.indexer = indexer;
		this.intakeRack = intakeRack;
		this.intakeRoller = intakeRoller;
		this.rollerFloor = rollerFloor;
		this.shooter = shooter;

		shootTimer.reset();
	}

	public void setRobotState(RobotState wantedState) {
		state = wantedState;

		if (!state.equals(RobotState.DRIVING_SCORING) && !state.equals(RobotState.DRIVING_SCORINGMOVING) && !state.equals(RobotState.DRIVING_IDLE)) {
			shootTimer.stop();
			shootTimer.reset();
		}
	}

	private void setSubsystemStates(DriveState driveState, HoodState hoodState, IndexerState indexerState,
			IntakeRackState intakeRackState, IntakeRollerState intakeRollerState, RollerFloorState rollerFloorState,
			ShooterState shooterState) {

		drivetrain.setDriveState(driveState);
		hood.setState(hoodState);
		indexer.setState(indexerState);
		intakeRack.setState(intakeRackState);
		intakeRoller.setState(intakeRollerState);
		rollerFloor.setState(rollerFloorState);
		shooter.setState(shooterState);
	}

	@Override
	public void periodic() {

		double intakeRackPosition;

		switch (state) {
			case STOPPED:
				this.setSubsystemStates(
					DriveState.BRAKE, 
					HoodState.STOP, 
					IndexerState.STOP, 
					IntakeRackState.STOP, 
					IntakeRollerState.STOP, 
					RollerFloorState.STOP, 
					ShooterState.STOP);
				break;

			case IDLE:
				this.setSubsystemStates(
					DriveState.IDLE, 
					HoodState.IDLE, 
					IndexerState.IDLE, 
					IntakeRackState.IDLE, 
					IntakeRollerState.IDLE, 
					RollerFloorState.IDLE, 
					ShooterState.IDLE);
				break;

			case DRIVING_STOWED:
				this.setSubsystemStates(
					DriveState.DRIVE_JOYSTICKS, 
					HoodState.TRENCH, 
					IndexerState.STOP, 
					IntakeRackState.STOWED_FIRING, 
					IntakeRollerState.STOP, 
					RollerFloorState.STOP, 
					ShooterState.FOLLOW_AUTOAIM_SLOW);
				break;

			case DRIVING_IDLE:
				this.setSubsystemStates(
					DriveState.DRIVE_JOYSTICKS, 
					HoodState.TRENCH, 
					IndexerState.STOP, 
					IntakeRackState.STOP, 
					IntakeRollerState.STOP, 
					RollerFloorState.STOP, 
					ShooterState.FOLLOW_AUTOAIM_SLOW);
				break;

			case DRIVING_INTAKING:
				this.setSubsystemStates(
					DriveState.DRIVE_JOYSTICKS, 
					HoodState.TRENCH, 
					IndexerState.STOP, 
					IntakeRackState.EXTENDED_FULL, 
					IntakeRollerState.FORWARD_FULLSPEED, 
					RollerFloorState.STOP, 
					ShooterState.FOLLOW_AUTOAIM_SLOW);
				break;

			case DRIVING_INTAKEDEPLOYED:
				this.setSubsystemStates(
					DriveState.DRIVE_JOYSTICKS, 
					HoodState.TRENCH, 
					IndexerState.STOP, 
					IntakeRackState.EXTENDED_FULL, 
					IntakeRollerState.STOP, 
					RollerFloorState.STOP, 
					ShooterState.FOLLOW_AUTOAIM_SLOW);
				break;

			case DRIVING_REVERSEINTAKING:
				this.setSubsystemStates(
					DriveState.DRIVE_JOYSTICKS, 
					HoodState.TRENCH, 
					IndexerState.REVERSE_SLOW, 
					IntakeRackState.EXTENDED_FULL, 
					IntakeRollerState.REVERSE_FULLSPEED, 
					RollerFloorState.REVERSE_FULLSPEED, 
					ShooterState.FOLLOW_AUTOAIM_SLOW);
				break;

			case DRIVING_SCORING:

				if (RobotConstants.isRedAlliance.getAsBoolean()) {
					if (drivetrain.getPose().getX() < 11) {
						autoaim.setFiringLocation(FiringLocation.PASS);
					} else {
						autoaim.setFiringLocation(FiringLocation.HUB);
					}
				} else {
					if (drivetrain.getPose().getX() > 5.57) {
						autoaim.setFiringLocation(FiringLocation.PASS);
					} else {
						autoaim.setFiringLocation(FiringLocation.HUB);
					}
				}

				drivetrain.setTargetPose(new Pose2d(drivetrain.getPose().getTranslation(), autoaim.getFieldRelativeAngleToFireWhileMoving()));

				drivetrain.setDriveState(DriveState.DRIVE_PIDTOANGLE);
				hood.setState(HoodState.FOLLOW_AUTOAIM);
				shooter.setState(ShooterState.FOLLOW_AUTOAIM);
				intakeRoller.setState(IntakeRollerState.FORWARD_HALFSPEED);

				if (autoaim.target.equals(FiringLocation.HUB) && 
					shooter.atSpeedToHubFire() &&
					hood.atPositionToFire() &&
					drivetrain.atAngleToScore()) {

					indexer.setState(IndexerState.FORWARD_FULLSPEED);
					rollerFloor.setState(RollerFloorState.FORWARD_FULLSPEED);

					shootTimer.start();
				} else if (autoaim.target.equals(FiringLocation.PASS) && 
					shooter.atSpeedToPassFire() &&
					hood.atPositionToFire() &&
					drivetrain.atAngleToScore()) {

					indexer.setState(IndexerState.FORWARD_FULLSPEED);
					rollerFloor.setState(RollerFloorState.FORWARD_FULLSPEED);

					shootTimer.start();
				} else {
					indexer.setState(IndexerState.STOP);
					rollerFloor.setState(RollerFloorState.STOP);

					shootTimer.stop();
				}

				intakeRackPosition = Math.max(
					SuperstructureConstants.IntakeRackConstants.INTAKERACK_MAX - 
					(SuperstructureConstants.IntakeRackConstants.INTAKERACK_MAX * shootTimer.get() * 0.5),
					SuperstructureConstants.IntakeRackConstants.INTAKERACK_MAX * 0.1);

				intakeRack.setOverridePosition(intakeRackPosition);
				intakeRack.setState(IntakeRackState.FOLLOW_OVERRIDE);
				break;

				
			case DRIVING_SCORINGMOVING:
				if (RobotConstants.isRedAlliance.getAsBoolean()) {
					if (drivetrain.getPose().getX() < 11) {
						autoaim.setFiringLocation(FiringLocation.PASS);
					} else {
						autoaim.setFiringLocation(FiringLocation.HUB);
					}
				} else {
					if (drivetrain.getPose().getX() > 5.57) {
						autoaim.setFiringLocation(FiringLocation.PASS);
					} else {
						autoaim.setFiringLocation(FiringLocation.HUB);
					}
				}

				drivetrain.setTargetPose(new Pose2d(drivetrain.getPose().getTranslation(), autoaim.getFieldRelativeAngleToFireWhileMoving()));

				drivetrain.setDriveState(DriveState.DRIVE_PIDTOANGLE_JOYSTICKS);
				hood.setState(HoodState.FOLLOW_AUTOAIM);
				shooter.setState(ShooterState.FOLLOW_AUTOAIM);
				intakeRoller.setState(IntakeRollerState.FORWARD_HALFSPEED);

				if (autoaim.target.equals(FiringLocation.HUB) && 
					shooter.atSpeedToHubFire() &&
					hood.atPositionToFire() &&
					drivetrain.atAngleToScore()) {

					indexer.setState(IndexerState.FORWARD_FULLSPEED);
					rollerFloor.setState(RollerFloorState.FORWARD_FULLSPEED);

					shootTimer.start();
				} else if (autoaim.target.equals(FiringLocation.PASS) && 
					shooter.atSpeedToPassFire() &&
					hood.atPositionToFire() &&
					drivetrain.atAngleToScore()) {

					indexer.setState(IndexerState.FORWARD_FULLSPEED);
					rollerFloor.setState(RollerFloorState.FORWARD_FULLSPEED);

					shootTimer.start();
				} else {
					indexer.setState(IndexerState.STOP);
					rollerFloor.setState(RollerFloorState.STOP);

					shootTimer.stop();
				}

				intakeRackPosition = Math.max(
					SuperstructureConstants.IntakeRackConstants.INTAKERACK_MAX - 
					(SuperstructureConstants.IntakeRackConstants.INTAKERACK_MAX * shootTimer.get() * 0.5),
					SuperstructureConstants.IntakeRackConstants.INTAKERACK_MAX * 0.1);

				intakeRack.setOverridePosition(intakeRackPosition);
				intakeRack.setState(IntakeRackState.FOLLOW_OVERRIDE);
				break;

			default:
				break;
		}
	}
}
