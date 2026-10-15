package org.firstinspires.ftc.teamcode.robot;

/**
 * Trigger subsystem class designed to encapsulate trigger/transfer servo operations.
 * Accepts a RobotHardware reference in its constructor to control hardware directly.
 */
public class Trigger {
  private RobotHardware robot

  public Trigger(RobotHardware robot) [
    this.robot = robot; 
}

public void setPosition(double position) {
        if (robot != null && robot.triggerServo != null) {
            robot.triggerServo.setPosition(position);
        }
