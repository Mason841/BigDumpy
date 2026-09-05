package frc.robot.constants;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class SuperstructureConstants {

    public class IDs {
        public static final int SHOOTER_TOP_RIGHT_MOTOR_ID = 0;
        public static final int SHOOTER_BOTTOM_RIGHT_MOTOR_ID = 0;
        public static final int SHOOTER_TOP_LEFT_MOTOR_ID = 0;
        public static final int SHOOTER_BOTTOM_LEFT_MOTOR_ID = 0;
        public static final int HOOD_MOTOR_ID = 0;
        public static final int INDEXER_RIGHT_MOTOR_ID = 0;
        public static final int INDEXER_LEFT_MOTOR_ID = 0;
        public static final int ROLLERFLOOR_RIGHT_MOTOR_ID = 0;
        public static final int ROLLERFLOOR_LEFT_MOTOR_ID = 0;
        public static final int INTAKERACK_RIGHT_MOTOR_ID = 0;
        public static final int INTAKERACK_LEFT_MOTOR_ID = 0;
        public static final int INTAKEROLLER_RIGHT_MOTOR_ID = 0;
        public static final int INTAKEROLLER_LEFT_MOTOR_ID = 0;
    }

    public class ShooterConstants {
        public static final Slot0Configs SHOOTER_PID_CONFIGS = new Slot0Configs()
            .withKP(0.3)
            .withKI(0)
            .withKD(0)
            .withKV(0.12)
            .withKA(0.01)
            .withKS(0.1)
            .withKG(0);

        public static final TalonFXConfiguration SHOOTER_MOTOR_CONFIGS = new TalonFXConfiguration()
            .withSlot0(SHOOTER_PID_CONFIGS)
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(60)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(70)
                .withSupplyCurrentLimitEnable(true))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withRotorToSensorRatio(1))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(100)
                    .withMotionMagicAcceleration(1000)
                    .withMotionMagicJerk(4000)
                    .withMotionMagicExpo_kA(0.1)
                    .withMotionMagicExpo_kV(0.12))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withAudio(new AudioConfigs().withAllowMusicDurDisable(true));
    }

    public class HoodConstants {

        public static final double HOOD_MIN = 0;
        public static final double HOOD_MAX = 10.85;

        public static final Slot0Configs HOOD_PID_CONFIGS = new Slot0Configs()
            .withKP(3)
            .withKI(0)
            .withKD(0)
            .withKV(0.12)
            .withKA(0)
            .withKS(0)
            .withKG(0);

        public static final TalonFXConfiguration HOOD_MOTOR_CONFIGS = new TalonFXConfiguration()
            .withSlot0(HOOD_PID_CONFIGS)
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(30)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withRotorToSensorRatio(1))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(120)
                    .withMotionMagicAcceleration(400)
                    .withMotionMagicJerk(4000)
                    .withMotionMagicExpo_kA(0.1)
                    .withMotionMagicExpo_kV(0.12))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withReverseSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(HOOD_MAX)
                    .withReverseSoftLimitThreshold(HOOD_MIN))
            .withAudio(new AudioConfigs().withAllowMusicDurDisable(true));
    }

    public class IndexerConstants {
        public static final Slot0Configs INDEXER_PID_CONFIGS = new Slot0Configs()
            .withKP(0.3)
            .withKI(0)
            .withKD(0)
            .withKV(0.12)
            .withKA(0.01)
            .withKS(0.1)
            .withKG(0);

        public static final TalonFXConfiguration INDEXER_MOTOR_CONFIGS = new TalonFXConfiguration()
            .withSlot0(INDEXER_PID_CONFIGS)
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(60)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(70)
                .withSupplyCurrentLimitEnable(true))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withRotorToSensorRatio(1))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(100)
                    .withMotionMagicAcceleration(1000)
                    .withMotionMagicJerk(4000)
                    .withMotionMagicExpo_kA(0.1)
                    .withMotionMagicExpo_kV(0.12))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withAudio(new AudioConfigs().withAllowMusicDurDisable(true));
    }

    public class RollerFloorConstants {
        public static final Slot0Configs ROLLERFLOOR_PID_CONFIGS = new Slot0Configs()
            .withKP(0.3)
            .withKI(0)
            .withKD(0)
            .withKV(0.12)
            .withKA(0.01)
            .withKS(0.1)
            .withKG(0);

        public static final TalonFXConfiguration ROLLERFLOOR_MOTOR_CONFIGS = new TalonFXConfiguration()
            .withSlot0(ROLLERFLOOR_PID_CONFIGS)
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(60)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(70)
                .withSupplyCurrentLimitEnable(true))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withRotorToSensorRatio(1))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(120)
                    .withMotionMagicAcceleration(1000)
                    .withMotionMagicJerk(4000)
                    .withMotionMagicExpo_kA(0.1)
                    .withMotionMagicExpo_kV(0.12))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withAudio(new AudioConfigs().withAllowMusicDurDisable(true));
    }

    public class IntakeRackConstants {
        public static final double INTAKERACK_MIN = 0;
        public static final double INTAKERACK_MAX = 10.68;

        public static final Slot0Configs INTAKERACK_PID_CONFIGS = new Slot0Configs()
            .withKP(3)
            .withKI(0)
            .withKD(0)
            .withKV(0.12)
            .withKA(0)
            .withKS(0)
            .withKG(0);

        public static final TalonFXConfiguration INTAKERACK_MOTOR_CONFIGS = new TalonFXConfiguration()
            .withSlot0(INTAKERACK_PID_CONFIGS)
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(30)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withRotorToSensorRatio(1))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(100)
                    .withMotionMagicAcceleration(400)
                    .withMotionMagicJerk(4000)
                    .withMotionMagicExpo_kA(0.1)
                    .withMotionMagicExpo_kV(0.12))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withReverseSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(INTAKERACK_MAX)
                    .withReverseSoftLimitThreshold(INTAKERACK_MIN))
            .withAudio(new AudioConfigs().withAllowMusicDurDisable(true));
    }
    
    public class IntakeRollerConstants {
        public static final Slot0Configs INTAKEROLLER_PID_CONFIGS = new Slot0Configs()
            .withKP(0.3)
            .withKI(0)
            .withKD(0)
            .withKV(0.12)
            .withKA(0.01)
            .withKS(0.1)
            .withKG(0);

        public static final TalonFXConfiguration INTAKEROLLER_MOTOR_CONFIGS = new TalonFXConfiguration()
            .withSlot0(INTAKEROLLER_PID_CONFIGS)
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(60)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(70)
                .withSupplyCurrentLimitEnable(true))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withRotorToSensorRatio(1))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(100)
                    .withMotionMagicAcceleration(1000)
                    .withMotionMagicJerk(4000)
                    .withMotionMagicExpo_kA(0.1)
                    .withMotionMagicExpo_kV(0.12))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withAudio(new AudioConfigs().withAllowMusicDurDisable(true));
    }
}