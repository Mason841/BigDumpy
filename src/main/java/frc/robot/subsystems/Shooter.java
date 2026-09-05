// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SuperstructureConstants;

public class Shooter extends SubsystemBase {

	private TalonFX topLeftMotorLeader = new TalonFX(SuperstructureConstants.IDs.SHOOTER_TOP_LEFT_MOTOR_ID);
	private TalonFX bottomLeftMotor = new TalonFX(SuperstructureConstants.IDs.SHOOTER_BOTTOM_LEFT_MOTOR_ID);
	private TalonFX topRightMotor = new TalonFX(SuperstructureConstants.IDs.SHOOTER_TOP_RIGHT_MOTOR_ID);
	private TalonFX bottomRightMotor = new TalonFX(SuperstructureConstants.IDs.SHOOTER_BOTTOM_RIGHT_MOTOR_ID);

	private Follower followerOpposed = new Follower(SuperstructureConstants.IDs.SHOOTER_TOP_LEFT_MOTOR_ID, MotorAlignmentValue.Opposed);
	private Follower followerAligned = new Follower(SuperstructureConstants.IDs.SHOOTER_TOP_LEFT_MOTOR_ID, MotorAlignmentValue.Aligned);

	private MotionMagicVelocityVoltage velocityControl = new MotionMagicVelocityVoltage(0);

	private SlewRateLimiter rpsLimiter = new SlewRateLimiter(10);

	public enum ShooterState {
		STOP,
		IDLE,
		FOLLOW_AUTOAIM,
		FOLLOW_AUTOAIM_SLOW,
		FOLLOW_OVERRIDE,
		PRESET_TRENCH,
		PRESET_TOWER
	}

	private ShooterState state = ShooterState.IDLE;

	private double overrideVelocity = 0;

	private double targetVelocity = 0;

	private DoubleSupplier autoaimShootSpeedSupplier;

	StatusCode[] latestStatus;

	public Shooter(DoubleSupplier autoaimShootSpeedSupplier) {
		topLeftMotorLeader.getConfigurator().apply(SuperstructureConstants.ShooterConstants.SHOOTER_MOTOR_CONFIGS);
		bottomLeftMotor.getConfigurator().apply(SuperstructureConstants.ShooterConstants.SHOOTER_MOTOR_CONFIGS);
		topRightMotor.getConfigurator().apply(SuperstructureConstants.ShooterConstants.SHOOTER_MOTOR_CONFIGS);
		bottomRightMotor.getConfigurator().apply(SuperstructureConstants.ShooterConstants.SHOOTER_MOTOR_CONFIGS);

		bottomLeftMotor.setControl(followerAligned);
		topRightMotor.setControl(followerOpposed);
		bottomRightMotor.setControl(followerOpposed);

		this.autoaimShootSpeedSupplier = autoaimShootSpeedSupplier;
	}

	public void setState(ShooterState wantedState) {
		state = wantedState;
		if (state.equals(ShooterState.FOLLOW_AUTOAIM_SLOW)) {
			rpsLimiter.reset(autoaimShootSpeedSupplier.getAsDouble());
		}
	}

	public void setOverrideVelocity(double wantedVelocity) {
		overrideVelocity = wantedVelocity;
	}

	public ShooterState getState() {
		return state;
	}

	public double getVelocity() {
		return this.topLeftMotorLeader.getRotorVelocity().getValueAsDouble();
	}

	public double getTargetVelocity() {
		return this.targetVelocity;
	}

	public void stopMotors() {
		this.targetVelocity = 0;
		this.topLeftMotorLeader.stopMotor();
	}

	public boolean atSpeedToHubFire() {
		return Math.abs(getTargetVelocity() - getVelocity()) < 3.0;
	}

	public boolean atSpeedToPassFire() {
		return Math.abs(getTargetVelocity() - getVelocity()) < 6.0;
	}

	public StatusCode[] setControl(ControlRequest control) {
		return new StatusCode[] {
				this.topLeftMotorLeader.setControl(control)
		};
	}

	@Override
	public void periodic() {
		Logger.recordOutput("Shooter/Velocity", this.getVelocity());
		Logger.recordOutput("Shooter/TargetVelocity", this.getTargetVelocity());
		Logger.recordOutput("Shooter/State", this.getState());

		switch (state) {
			case STOP:
				targetVelocity = 0;
				break;

			case IDLE:
				stopMotors();
				break;

			case FOLLOW_AUTOAIM:
				targetVelocity = autoaimShootSpeedSupplier.getAsDouble();
				break;

			case FOLLOW_AUTOAIM_SLOW:
				targetVelocity = rpsLimiter.calculate(autoaimShootSpeedSupplier.getAsDouble());
				break;

			case FOLLOW_OVERRIDE:
				targetVelocity = overrideVelocity;
				break;

			case PRESET_TRENCH:
				targetVelocity = 60;
				break;

			case PRESET_TOWER:
				targetVelocity = 50;
				break;
		
			default:
				this.stopMotors();
				break;
		}

		if (targetVelocity == 0 && !(state.equals(ShooterState.STOP))) {
			this.stopMotors();
		} else {
			latestStatus = this.setControl(velocityControl.withVelocity(targetVelocity));
		}
	}
}
