package org.firstinspires.ftc.teamcode.robot;
public class HiveVision {
//variables

//DO NOT DECLARE MOTORS HERE
// when calling motors, use robot.MOTORNAME.setPower or smth

public HiveVision (RobotHardware robot) {
        this.robot = robot;
    }

    private void getAllResults() {
        limelight.setPipeline(0);

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            List<FiducialResult> fiducials = result.getFiducialResults();

            for (FiducialResult fiducial : fiducials) {
                int tagId = fiducial.getFiducialId(); // Retrieve AprilTag ID

                if (tagId == 1) {
                    double tx = fiducial.getTargetXDegrees();
                    double ty = fiducial.getTargetYDegrees();

                } else if (tagId == 2) {
                    double distance = fiducial.getRobotPoseTargetSpace()[2]; // Z-axis distance

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
        return hiveTipState; //hiveTipState is either “GARDEN_SIDE” or “OPPOSITE_GARDEN”
    }

    public Pose3D getAverageBotPose() {
        //return averaged botposes
    }
}
