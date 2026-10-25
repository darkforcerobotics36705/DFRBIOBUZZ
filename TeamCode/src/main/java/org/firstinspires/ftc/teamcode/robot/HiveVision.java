package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;


public class HiveVision {

    private RobotHardware robot;

//DO NOT DECLARE MOTORS HERE
// when calling motors, use robot.MOTORNAME.setPower or smth

public HiveVision (RobotHardware robot) {
        this.robot = robot;

    }

    private void getAllResults() {
        robot.limelight.pipelineSwitch(0);

        LLResult result = robot.limelight.getLatestResult();

        if (result != null && result.isValid()) {

            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int tagId = fiducial.getFiducialId(); // Retrieve AprilTag ID

                if (tagId == 1) {
                    double tx = fiducial.getTargetXDegrees();
                    double ty = fiducial.getTargetYDegrees();

                } else if (tagId == 2) {
                    double distance = fiducial.getRobotPoseTargetSpace()
                            .getPosition().z;

                } else {
                    // Handle other tags
                    telemetry.addData("Seen Unhandled Tag ID", tagId);
                }
            }
        }
    }

    public double getAverageTa() {
        //average the areas
    }

    public double getTargetHeading() {
        //tx from custom defined point, transformed to field centric heading
    }

    public void updateRead(String allianceColor) {
        //get latest results
        //call updateHiveTipstate
    }

    private void updateHiveTipState() {
        //compare average ta and probably better to use ty, with ID of the tags to determine which side of the field we are on, then update it
    }

    public String getHiveTipState() {
        String hiveTipState;
        //hiveTipState is either “GARDEN_SIDE” or “OPPOSITE_GARDEN”
    }

    public Pose3D getAverageBotPose() {

    }
}
