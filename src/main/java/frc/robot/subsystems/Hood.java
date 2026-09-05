// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SuperstructureConstants;

public class Hood extends SubsystemBase {

	private TalonFX motor = new TalonFX(SuperstructureConstants.IDs.HOOD_MOTOR_ID);

	private MotionMagicExpoVoltage positionControl = new MotionMagicExpoVoltage(0);

	public enum HoodState {
		STOP,
		IDLE,
		TRENCH,
		TRENCH_SLAM,
		FOLLOW_AUTOAIM,
		FOLLOW_OVERRIDE,
		PRESET_TRENCH,
		PRESET_TOWER
	}

	private HoodState state = HoodState.TRENCH;

	private double overridePosition = 0;

	private double targetPosition = 0;

	private DoubleSupplier autoaimHoodPositionSupplier;

	StatusCode[] latestStatus;

	public Hood(DoubleSupplier autoaimHoodPositionSupplier) {
		motor.getConfigurator().apply(SuperstructureConstants.HoodConstants.HOOD_MOTOR_CONFIGS);
		this.autoaimHoodPositionSupplier = autoaimHoodPositionSupplier;
	}

	public void setState(HoodState wantedState) {
		state = wantedState;
	}

	public void setOverridePosition(double wantedPosition) {
		overridePosition = wantedPosition;
	}

	public HoodState getState() {
		return state;
	}

	public double getPosition() {
		return this.motor.getPosition().getValueAsDouble();
	}

	public double getTargetPosition() {
		return this.targetPosition;
	}

	public void stopMotor() {
		this.motor.stopMotor();
	}

	public boolean atPositionToFire() {
		return Math.abs(getTargetPosition() - getPosition()) < 1.0 && (!state.equals(HoodState.TRENCH) && !state.equals(HoodState.TRENCH_SLAM));
	}

	public StatusCode[] setControl(ControlRequest control) {
		return new StatusCode[] {
				this.motor.setControl(control)
		};
	}

	@Override
	public void periodic() {
		Logger.recordOutput("Hood/Position", this.getPosition());
		Logger.recordOutput("Hood/TargetPosition", this.getTargetPosition());
		Logger.recordOutput("Hood/State", this.getState());

		switch (state) {
			case STOP:
				break;
			
			case IDLE:
				stopMotor();
				break;

			case TRENCH:
				targetPosition = SuperstructureConstants.HoodConstants.HOOD_MIN;
				break;

			case TRENCH_SLAM:
				targetPosition = SuperstructureConstants.HoodConstants.HOOD_MIN;
				break;

			case FOLLOW_AUTOAIM:
				targetPosition = autoaimHoodPositionSupplier.getAsDouble();
				break;

			case FOLLOW_OVERRIDE:
				targetPosition = overridePosition;
				break;
				
			case PRESET_TRENCH:
				targetPosition = 15;
				break;

			case PRESET_TOWER:
				targetPosition = 20;
				break;

			default:
				targetPosition = SuperstructureConstants.HoodConstants.HOOD_MIN;
				break;
		}

		if (state.equals(HoodState.IDLE)) {
			stopMotor();
		} else {
			latestStatus = this.setControl(positionControl.withPosition(targetPosition));
		}
	}
}
