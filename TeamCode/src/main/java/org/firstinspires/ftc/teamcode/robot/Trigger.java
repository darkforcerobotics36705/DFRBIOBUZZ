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
            robot.triggerMotor.setPower(position);
        }
 public void trigger() {
        robot.triggerMotor.setPower(RobotConfig.TRIGGER_FIRING_POWER);
    }

    /**
     * Convenience method to reset/retract the trigger mechanism.
     */
    public void reset() {
        robot.triggerMotor.setPower(RobotConfig.TRIGGER_FIRING_POWER);
    }

    /**
     * Retrieve current position of the trigger servo.
     * @return Current servo position or 0.0 if null.
     */
    public double getPosition() {
        if (robot != null && robot.triggerServo != null) {
            return robot.triggerMotor.getPosition();
        }
        return 0.0;
    }
}
