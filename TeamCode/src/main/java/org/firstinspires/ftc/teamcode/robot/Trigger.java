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
 public void trigger() {
        setPosition(0.5);
    }

    /**
     * Convenience method to reset/retract the trigger mechanism.
     */
    public void reset() {
        setPosition(0.0);
    }

    /**
     * Retrieve current position of the trigger servo.
     * @return Current servo position or 0.0 if null.
     */
    public double getPosition() {
        if (robot != null && robot.triggerServo != null) {
            return robot.triggerServo.getPosition();
        }
        return 0.0;
    }
}
