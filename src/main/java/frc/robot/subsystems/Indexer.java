// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SuperstructureConstants;

public class Indexer extends SubsystemBase {

  	private TalonFX leftMotorLeader = new TalonFX(SuperstructureConstants.IDs.INDEXER_LEFT_MOTOR_ID);
	private TalonFX rightMotor = new TalonFX(SuperstructureConstants.IDs.INDEXER_RIGHT_MOTOR_ID);

	private Follower followerOpposed = new Follower(SuperstructureConstants.IDs.INDEXER_LEFT_MOTOR_ID, MotorAlignmentValue.Opposed);

	private MotionMagicVelocityVoltage velocityControl = new MotionMagicVelocityVoltage(0);

	public enum IndexerState {
		STOP,
		IDLE,
		FORWARD_FULLSPEED,
		FORWARD_HALFSPEED,
		FORWARD_INDEXSLOW,
		REVERSE_FULLSPEED,
		REVERSE_SLOW,
		FOLLOW_OVERRIDE
	}

	private IndexerState state = IndexerState.STOP;

	private double overrideVelocity = 0;

	private double targetVelocity = 0;

	StatusCode[] latestStatus;

	public Indexer() {
		leftMotorLeader.getConfigurator().apply(SuperstructureConstants.IndexerConstants.INDEXER_MOTOR_CONFIGS);
		rightMotor.getConfigurator().apply(SuperstructureConstants.IndexerConstants.INDEXER_MOTOR_CONFIGS);

		rightMotor.setControl(followerOpposed);
	}

	public void setState(IndexerState wantedState) {
		state = wantedState;
	}

	public void setOverrideVelocity(double wantedVelocity) {
		overrideVelocity = wantedVelocity;
	}

	public IndexerState getState() {
		return state;
	}

	public double getVelocity() {
		return this.leftMotorLeader.getRotorVelocity().getValueAsDouble();
	}

	public double getTargetVelocity() {
		return this.targetVelocity;
	}

	public void stopMotors() {
		this.targetVelocity = 0;
		this.leftMotorLeader.stopMotor();
	}

	public StatusCode[] setControl(ControlRequest control) {
		return new StatusCode[] {
			this.leftMotorLeader.setControl(control)
		};
	}

	@Override
	public void periodic() {
		Logger.recordOutput("Indexer/Velocity", this.getVelocity());
		Logger.recordOutput("Indexer/TargetVelocity", this.getTargetVelocity());
		Logger.recordOutput("Indexer/State", this.getState());

		switch (state) {
			case STOP:
				targetVelocity = 0;
				break;

			case IDLE:
				stopMotors();
				break;

			case FORWARD_FULLSPEED:
				targetVelocity = 100;
				break;

			case FORWARD_HALFSPEED:
				targetVelocity = 50;
				break;

			case FORWARD_INDEXSLOW:
				targetVelocity = 10;
				break;

			case REVERSE_FULLSPEED:
				targetVelocity = -50;
				break;

			case REVERSE_SLOW:
				targetVelocity = -15;
				break;

			case FOLLOW_OVERRIDE:
				targetVelocity = overrideVelocity;
				break;

			default:
				this.stopMotors();
				break;
		}

		if (targetVelocity == 0 && !(state.equals(IndexerState.STOP))) {
			this.stopMotors();
		} else {
			latestStatus = this.setControl(velocityControl.withVelocity(targetVelocity));
		}
	}
}
