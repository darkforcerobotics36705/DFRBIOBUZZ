package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

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
    }

    public void updateRead(String allianceColor) {
        //get latest results
        call updateHiveTipState;
    }

    private void updateHiveTipState() {
        //compare average ta and probably better to use ty, with ID of the tags to determine which side of the field we are on, then update it
    }

    public String getHiveTipState() {
        String hiveTipState;
        //hiveTipState is either “GARDEN_SIDE” or “OPPOSITE_GARDEN”
    }

    public Pose3D getAverageBotPose() {
        Pose3d.botPose = robot.limelight.botPose;
        double x = botPose.getPosition().x;
        double y = botPose.getPosition().y;
        double z = botPose.getposition().z;

        telemetry.addData("Robot X", x);
        telemetry.addData("Robot Y", y);
        telemetry.addData("Robot Z", z);
        telemetry.update();
    }
}
