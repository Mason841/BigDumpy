package frc.robot.subsystems.vision;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

    private final VisionIO io;
    private final VisionIOInputs inputs = new VisionIOInputs();

    private final Drivetrain drivetrain;

    public static final Vector<N3> standardVisionDevs2OrMore = VecBuilder.fill(0.3, 0.3, 0.02);
    public static final Vector<N3> standardVisionDevs1tag = VecBuilder.fill(0.5, 0.5, 0.5);

    public static final Vector<N3> standardVisionDevsMT2 = VecBuilder.fill(1, 1, 999);
    public static final Vector<N3> standardVisionDevsMT1 = VecBuilder.fill(999, 999, 1);

    public boolean disableVision = false;
    public boolean disableTurretCamera = false;

    public Vision(VisionIO io, Drivetrain drivetrain) {
        this.io = io;
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        double timestamp = Timer.getTimestamp();
        SwerveDrivetrain.SwerveDriveState lastDriveState = this.drivetrain.getState();
        inputs.robotYawDegrees = lastDriveState.Pose.getRotation().getMeasure().in(Units.Degree);
        inputs.robotYawRateDegreesPerSecond = Math.toDegrees(lastDriveState.Speeds.omegaRadiansPerSecond);
        inputs.robotRollDegrees = this.drivetrain.getPigeon2().getRoll().getValue().in(Units.Degree);
        inputs.robotRollRateDegreesPerSecond = 0;
        inputs.robotPitchDegrees = this.drivetrain.getPigeon2().getPitch().getValue().in(Units.Degree);
        inputs.robotPitchRateDegreesPerSecond = 0;

        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);

        boolean reject = false;

        if (Math.abs(lastDriveState.Speeds.omegaRadiansPerSecond) > 3) {
            reject = true;
        }

        if (disableVision) {
            reject = true;
        }

        if (!reject) {
            try {
                if (inputs.limelightHasTarget) {
                    filterLL(
                        inputs.limelightPoseEstimateMT1.pose(), 
                        inputs.limelightPoseEstimateMT1.tagCount(), 
                        inputs.limelightPoseEstimateMT1.avgTagArea(), 
                        inputs.limelightPoseEstimateMT1.timestampSeconds(), 
                        false);
                    filterLL(
                        inputs.limelightPoseEstimateMT2.pose(), 
                        inputs.limelightPoseEstimateMT2.tagCount(), 
                        inputs.limelightPoseEstimateMT2.avgTagArea(), 
                        inputs.limelightPoseEstimateMT2.timestampSeconds(), 
                        true);
                }
            } catch (Exception ignored) {
            }
        }

        Logger.recordOutput("Vision/latencyPeriodicSec", Timer.getTimestamp() - timestamp);
    }

    private void filterLL(Pose2d pose, int tagCount, double avgTagArea, double timestampSeconds, boolean isMT2) {
        if (tagCount > 0) {
            if (tagCount == 1 && isMT2) {
                this.drivetrain.addVisionMeasurement(
                        pose,
                        timestampSeconds,
                        standardVisionDevs1tag.elementTimes(standardVisionDevsMT2)
                );
            } else if (tagCount > 1 && isMT2) {
                this.drivetrain.addVisionMeasurement(
                        pose,
                        timestampSeconds,
                        standardVisionDevs2OrMore.elementTimes(standardVisionDevsMT2)
                );
            } else if (tagCount > 2 && !isMT2) {
                this.drivetrain.addVisionMeasurement(
                        pose,
                        timestampSeconds,
                        standardVisionDevs2OrMore.elementTimes(standardVisionDevsMT1)
                );
            }
        }
    }

    public void disableVision() {
        disableVision = true;
    }

    public void enableVision() {
        disableVision = false;
    }

    public void disableTurretVision() {
        disableTurretCamera = true;
    }

    public void enableTurretVision() {
        disableTurretCamera = false;
    }
}
