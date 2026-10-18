package org.firstinspires.ftc.teamcode.robot;
public class HiveVision {
//any variables here
//DO NOT DECLARE MOTORS HERE
// when calling motors, use robot.MOTORNAME.setPower or smth

public HiveVision (RobotHardware robot) {
        this.robot = robot;
}

    private void getAllResults() {
        //stuff
        //probably update a list
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
