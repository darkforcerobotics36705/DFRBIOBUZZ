package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Launcher {
    private RobotHardware robot;
    public Launcher(RobotHardware robot) {
        this.robot = robot;
    }
    private double vel;
    private double hoodRetractionPos;
    /*
    public void setLauncher() {
        vel = Math.sqrt(Math.pow(ShooterConfig.elementVelocity[0],2) + Math.pow(ShooterConfig.elementVelocity[1],2));
        setHoodExtension();
        setVelocityNormal(vel);
        //setVelocityBang(vel);
        //setVelocityPFPow(vel);
        //setVelocityPFVol(vel);
    }
    */

    public void setVelocityNormal(double vel) {
        robot.launchMotor1.setVelocity(vel);
        robot.launchMotor2.setVelocity(vel);
    }
    /*
    private void setVelocityBang(double vel) {
        if (currVelocity < vel) {
            robot.flywheelMotor.setPower(1.0);
        } else if (currVelocity > vel) {
            robot.flywheelMotor.setPower(0.0);
        }
    }
    private void setVelocityPFPow(double vel) {

    }
    private void setVelocityPFVol(double vel) {

    }
    */
}