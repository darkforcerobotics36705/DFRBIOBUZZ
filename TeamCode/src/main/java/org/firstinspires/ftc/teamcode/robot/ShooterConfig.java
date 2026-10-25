package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.robot.helperClasses.InterpLUT;

public class ShooterConfig {
    public static double currentRobotHeading;
    public static double[] elementConfig; //[double x, double y, double hoodPollenAngle, double hoodNectarAngle]
    public static double[] distance = {

    };
    public static double[] apriltagTa = {

    };
    public static double[] flywheelVelocity = {

    };
    // gotta have data on which rpm gives what velocity
    public static double[] elementVelocity = {

    };
    public static double[] pollenHoodAngle = {

    };
    public static double[] nectarHoodAngle = {

    };
    private static final InterpLUT distToVel = new InterpLUT(distance, flywheelVelocity);
    private static final InterpLUT apriltagTaToVel = new InterpLUT(apriltagTa, flywheelVelocity);
    private static final InterpLUT distToPollenAngle = new InterpLUT(distance, pollenHoodAngle);
    private static final InterpLUT distToNectarAngle = new InterpLUT(distance, nectarHoodAngle);
    public void calculateShootingConfig(double averageTa, double[] dist, double[] robotTranslationalVelocity, double targetLLHeading, double currRobotHeading) {
        //use the InterpLUT class to get the shooter velocity from averageTa and dist, then AVERAGE the two.

        double magnitude;
        double distance = Math.hypot(dist[0], dist[1]);
        if (averageTa != 0.0) {
            magnitude = 0.9 * (distToVel.get(distance)) + 0.1 * (apriltagTaToVel.get(averageTa));
        } else {
            magnitude = (distToVel.get(distance));
        }
        currentRobotHeading = currRobotHeading;

        //then do vector math for x and y
        /*
        * ============ EXTREME PROBLEM ==============
        * Shoot on the move vector math WONT WORK WITH RPM VELOCITY, you must convert distance to elementConfig
        * then do subtraction, then convert the new element velocity BACK into rpm.
        *
        * This should only affect magnitude, so angle stuffs should be fine
        */
        double pollenAngle = distToPollenAngle.get(distance);
        double nectarAngle = distToNectarAngle.get(distance);
        //correct the SOTM errors
        double vx = magnitude * dist[0] / distance;
        double vy = magnitude * dist[1] / distance;
        vx -= robotTranslationalVelocity[0];
        vy -= robotTranslationalVelocity[1];
        
        //update the elementConfig list
        elementConfig = new double[]{vx, vy, pollenAngle, nectarAngle};

    }
}
