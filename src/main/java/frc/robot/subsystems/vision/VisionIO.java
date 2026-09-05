package frc.robot.subsystems.vision;

import frc.robot.LimelightHelpers;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface VisionIO {
    class VisionIOInputs implements LoggableInputs {
        public boolean limelightHasTarget;

        public LimelightHelpers.RawFiducial[] limelightRawFiducial = new LimelightHelpers.RawFiducial[]{};
    
        public LimelightHelpers.PoseEstimate limelightPoseEstimateMT1 = new LimelightHelpers.PoseEstimate(false);
        public LimelightHelpers.PoseEstimate limelightPoseEstimateMT2 = new LimelightHelpers.PoseEstimate(true);

        public double robotYawDegrees;
        public double robotYawRateDegreesPerSecond;
        public double robotPitchDegrees;
        public double robotPitchRateDegreesPerSecond;
        public double robotRollDegrees;
        public double robotRollRateDegreesPerSecond;

        @Override
        public void toLog(LogTable table) {
            table.put("limelightHasTarget", limelightHasTarget);
         
            table.put("limelightRawFiducial", rawFiducialsToLogArray(limelightRawFiducial));
       
            logPoseEstimate(table, "limelightPoseEstimateMT1", limelightPoseEstimateMT1);
            logPoseEstimate(table, "limelightPoseEstimateMT2", limelightPoseEstimateMT2);
          
            table.put("robotYawDegrees", robotYawDegrees);
            table.put("robotYawRateDegreesPerSecond", robotYawRateDegreesPerSecond);
            table.put("robotPitchDegrees", robotPitchDegrees);
            table.put("robotPitchRateDegreesPerSecond", robotPitchRateDegreesPerSecond);
            table.put("robotRollDegrees", robotRollDegrees);
            table.put("robotRollRateDegreesPerSecond", robotRollRateDegreesPerSecond);
        }

        @Override
        public void fromLog(LogTable table) {
            limelightHasTarget = table.get("limelightHasTarget", limelightHasTarget);
          
            limelightRawFiducial = rawFiducialsFromLogArray(table.get("limelightRawFiducial", rawFiducialsToLogArray(limelightRawFiducial)));
          
            limelightPoseEstimateMT1 = readPoseEstimate(table, "limelightPoseEstimateMT1", limelightPoseEstimateMT1);
            limelightPoseEstimateMT2 = readPoseEstimate(table, "limelightPoseEstimateMT2", limelightPoseEstimateMT2);
           
            robotYawDegrees = table.get("robotYawDegrees", robotYawDegrees);
            robotYawRateDegreesPerSecond = table.get("robotYawRateDegreesPerSecond", robotYawRateDegreesPerSecond);
            robotPitchDegrees = table.get("robotPitchDegrees", robotPitchDegrees);
            robotPitchRateDegreesPerSecond = table.get("robotPitchRateDegreesPerSecond", robotPitchRateDegreesPerSecond);
            robotRollDegrees = table.get("robotRollDegrees", robotRollDegrees);
            robotRollRateDegreesPerSecond = table.get("robotRollRateDegreesPerSecond", robotRollRateDegreesPerSecond);
        }

        private static void logPoseEstimate(LogTable table, String key, LimelightHelpers.PoseEstimate estimate) {
            LogTable poseTable = table.getSubtable(key);
            poseTable.put("pose", estimate.pose());
            poseTable.put("timestampSeconds", estimate.timestampSeconds());
            poseTable.put("latency", estimate.latency());
            poseTable.put("tagCount", estimate.tagCount());
            poseTable.put("tagSpan", estimate.tagSpan());
            poseTable.put("avgTagDist", estimate.avgTagDist());
            poseTable.put("avgTagArea", estimate.avgTagArea());
            poseTable.put("rawFiducials", rawFiducialsToLogArray(estimate.rawFiducials()));
            poseTable.put("isMegaTag2", estimate.isMegaTag2());
        }

        private static LimelightHelpers.PoseEstimate readPoseEstimate(
                LogTable table, String key, LimelightHelpers.PoseEstimate defaultValue) {
            LogTable poseTable = table.getSubtable(key);
            return new LimelightHelpers.PoseEstimate(
                    poseTable.get("pose", defaultValue.pose()),
                    poseTable.get("timestampSeconds", defaultValue.timestampSeconds()),
                    poseTable.get("latency", defaultValue.latency()),
                    poseTable.get("tagCount", defaultValue.tagCount()),
                    poseTable.get("tagSpan", defaultValue.tagSpan()),
                    poseTable.get("avgTagDist", defaultValue.avgTagDist()),
                    poseTable.get("avgTagArea", defaultValue.avgTagArea()),
                    rawFiducialsFromLogArray(poseTable.get("rawFiducials", rawFiducialsToLogArray(defaultValue.rawFiducials()))),
                    poseTable.get("isMegaTag2", defaultValue.isMegaTag2()));
        }

        private static double[][] rawFiducialsToLogArray(LimelightHelpers.RawFiducial[] rawFiducials) {
            double[][] values = new double[rawFiducials.length][7];
            for (int i = 0; i < rawFiducials.length; i++) {
                values[i][0] = rawFiducials[i].id();
                values[i][1] = rawFiducials[i].txnc();
                values[i][2] = rawFiducials[i].tync();
                values[i][3] = rawFiducials[i].ta();
                values[i][4] = rawFiducials[i].distToCamera();
                values[i][5] = rawFiducials[i].distToRobot();
                values[i][6] = rawFiducials[i].ambiguity();
            }
            return values;
        }

        private static LimelightHelpers.RawFiducial[] rawFiducialsFromLogArray(double[][] values) {
            LimelightHelpers.RawFiducial[] rawFiducials = new LimelightHelpers.RawFiducial[values.length];
            for (int i = 0; i < values.length; i++) {
                double[] value = values[i];
                rawFiducials[i] = new LimelightHelpers.RawFiducial(
                        value.length > 0 ? (int) value[0] : 0,
                        value.length > 1 ? value[1] : 0.0,
                        value.length > 2 ? value[2] : 0.0,
                        value.length > 3 ? value[3] : 0.0,
                        value.length > 4 ? value[4] : 0.0,
                        value.length > 5 ? value[5] : 0.0,
                        value.length > 6 ? value[6] : 0.0);
            }
            return rawFiducials;
        }
    }

    void updateInputs(VisionIOInputs inputs);
}
