package org.firstinspires.ftc.teamcode.robot;

/**
 * Intake subsystem class designed to encapsulate intake motor operations.
 * Accepts a RobotHardware reference in its constructor to control hardware directly.
 */
public class Intake {
    private RobotHardware robot;

    
     //Constructor injecting the RobotHardware instance.
    public Intake(RobotHardware robot) {
        this.robot = robot;
    }

    
    public void setPower(double power) {
        if (robot != null && robot.intakeMotor != null) {
            robot.intakeMotor.setPower(power);
        }
    }

    /**
     * Convenience method to pull game elements into the robot at full power.
     */
    public void intake() {
        setPower(1.0);
    }

    /**
     * Convenience method to reverse/outtake game elements at full power.
     */
    public void outtake() {
        setPower(-1.0);
    }

    /**
     * Stop the intake motor.
     */
    public void stop() {
        setPower(0.0);
    }

    /**
     * Retrieve current power level of the intake motor.
     * @return Current motor power or 0.0 if null
     */
    public double getPower() {
        if (robot != null && robot.intakeMotor != null) {
            return robot.intakeMotor.getPower();
        }
        return 0.0;
    }
}
