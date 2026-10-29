package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import java.util.List;
import java.util.ArrayList;

public class TagData {
    public int id;
    public double tx;
    public double ty;
    public double distance;

    public class HiveVision {

        private RobotHardware robot;

        public HiveVision(RobotHardware robot) {
            this.robot = robot;

        }

        private void getAllResults() {
            robot.limelight.pipelineSwitch(0);

            LLResult result = robot.limelight.getLatestResult();

            if (result != null && result.isValid()) {

                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    int tagId = fiducial.getFiducialId(); // Retrieve AprilTag ID

                    //red (audience side)
                    if (tagId == 34 || tagId == 35 || tagId == 36 || tagId == 37) {
                        double tx = fiducial.getTargetXDegrees();
                        double ty = fiducial.getTargetYDegrees();
                        telemetry.addData("Target", "(Target " + tagId + ")");

                        //blue (audience side)
                    } else if (tagId == 38 || tagId == 39 || tagId == 40 || tagId == 41) {
                        double distance = fiducial.getRobotPoseTargetSpace()
                                .getPosition().z;
                        telemetry.addData("Target", "(Target " + tagId + ")");

                        //blue (opp. side)
                    } else if (tagId == 42 || tagId == 43 || tagId == 44 || tagId == 45) {
                        telemetry.addData("Target", "(Target " + tagId + ")");

                        //red (opp. side)
                    } else if (tagId == 0 || tagId == 1 || tagId == 2 || tagId == 3) {
                        telemetry.addData("Target", "(Target " + tagId + ")");
                    } else {
                        //catch other tags
                        telemetry.addData("Target", "(Target " + tagId + ")");
                    }
                }
            }
        }

        public double getAverageTa() {
            //average the areas
        }

        public double getTargetHeading() {
            double average = (tx + ty) / 2.0;
            return average;
        }

        public void updateRead(String allianceColor) {
            //get latest results
            getAllResults();
            updateHiveTipState();
        }

        private void updateHiveTipState() {
            //compare average ta and probably better to use ty, with ID of the tags to determine which side of the field we are on, then update it
        }

        public String getHiveTipState() {
            //check this not sure it's correct
            String hiveTipState = "some_SIDE";
            //hiveTipState is either “GARDEN_SIDE” or “OPPOSITE_GARDEN”
            robot.limelight.pipelineSwitch(0);

            LLResult result = robot.limelight.getLatestResult();

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fiducial : fiducials){
                    int tagId = fiducial.getFiducialId();

                    if (tagId == 30 || tagId == 31 || tagId == 32 || tagId == 33 ||
                            tagId == 42 || tagId == 43 || tagId == 44 || tagId == 45) {
                        hiveTipState = "OPPOSITE_GARDEN";
                        return hiveTipState;
                    } else {
                        hiveTipState = "GARDEN_SIDE";
                        return hiveTipState;
                    }
                }

            }

            return hiveTipState;
        }

        public Pose3D getAverageBotPose() {
            final int samples = 10;

            double totalX = 0;
            double totalY = 0;
            double totalZ = 0;

            int validSamples = 0;
            Pose3D latestPose = null;

            for (int i = 0; i < samples; i++) {
                LLResult result = robot.limelight.getLatestResult();

                if (result == null || !result.isValid()) {
                    continue;
                }

                Pose3D pose = result.getBotpose();

                if (pose == null || pose.getPosition() == null) {
                    continue;
                }

                totalX += pose.getPosition().x;
                totalY += pose.getPosition().y;
                totalZ += pose.getPosition().z;

                latestPose = pose;
                validSamples++;
            }

            if (validSamples == 0) {
                telemetry.addLine("No valid Limelight readings");
                telemetry.update();
                return null;
            }

            double avgX = totalX / validSamples;
            double avgY = totalY / validSamples;
            double avgZ = totalZ / validSamples;

            telemetry.addData("Average X", avgX);
            telemetry.addData("Average Y", avgY);
            telemetry.addData("Average Z", avgZ);
            telemetry.addData("Valid Samples", validSamples);
            telemetry.update();

            return new Pose3D(
                    new Position(
                            latestPose.getPosition().unit,
                            avgX,
                            avgY,
                            avgZ,
                            latestPose.getPosition().acquisitionTime
                    ),
                    latestPose.getOrientation()
            );
        }
    }
}
