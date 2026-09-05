package frc.robot.subsystems.vision;

import frc.robot.LimelightHelpers;
import frc.robot.constants.RobotConstants;

public class VisionIOLimelights implements VisionIO{
    VisionIOInputs inputCache = new VisionIOInputs();

    public VisionIOLimelights(){
        setLLSettings(inputCache);
    }

    private void setLLSettings(VisionIOInputs inputs) {
        try {
            
            LimelightHelpers.setCameraPose_RobotSpace(
                RobotConstants.Vision.limelightName, 
                RobotConstants.Vision.limelightPose[0], 
                RobotConstants.Vision.limelightPose[1], 
                RobotConstants.Vision.limelightPose[2], 
                RobotConstants.Vision.limelightPose[3], 
                RobotConstants.Vision.limelightPose[4], 
                RobotConstants.Vision.limelightPose[5]);
    
            LimelightHelpers.SetIMUMode(RobotConstants.Vision.limelightName, 0);
    
            LimelightHelpers.SetRobotOrientation_NoFlush(
                    RobotConstants.Vision.limelightName,
                    inputs.robotYawDegrees,
                    inputs.robotYawRateDegreesPerSecond,
                    inputs.robotPitchDegrees,
                    inputs.robotPitchRateDegreesPerSecond,
                    inputs.robotRollDegrees,
                    inputs.robotRollRateDegreesPerSecond
            );
            
            LimelightHelpers.Flush();
        } catch (Exception e) {
            System.out.println("Error setting limelight settings: " + e.getMessage());
        }
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        this.inputCache = inputs;
        setLLSettings(this.inputCache);

        inputs.limelightHasTarget = LimelightHelpers.getTV(RobotConstants.Vision.limelightName);

        if (inputs.limelightHasTarget) {
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(RobotConstants.Vision.limelightName);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(RobotConstants.Vision.limelightName);
            inputs.limelightPoseEstimateMT1 = mt1;
            inputs.limelightPoseEstimateMT2 = mt2;
            inputs.limelightRawFiducial = mt1.rawFiducials();
        }
    }
}
