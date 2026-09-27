package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Launcher {

    private RobotHardware robot;
    private ElapsedTime launchTimer = new ElapsedTime();
    private ElapsedTime tripleShotTimer = new ElapsedTime();

    public boolean tripleShotStarted = false;

    // State machines
    private enum LaunchState { IDLE, FIRING, RESETTING }
    private LaunchState launchState = LaunchState.IDLE;

    public enum TripleShotState { IDLE, TRANSFER_RUNNING, FIRING_BALL,
        WAITING_BETWEEN_SHOTS, RESETTING_TRIGGER, COMPLETE }
    private TripleShotState tripleShotState = TripleShotState.IDLE;

    // Internal state
    private double currentLaunchPower;
    private double tripleShotBasePower = 0.0;
    private int shotsFired = 0;
    private double presentVoltage = 12.3;

    public Launcher(RobotHardware robot) {
        this.robot = robot;
        currentLaunchPower = MIN_LAUNCH_POWER;
    }

    public void init() {
        robot.Trigger.setPosition(TRIGGER_START_POS);
    }

    /** Call every loop. Handles power, single shot, and triple shot. */
    public void update(double rightTrigger, boolean tripleShot,
                       boolean hasTarget, double distance, double voltage) {

        this.presentVoltage = voltage;

        // --- Update launch power based on distance ---
        if (hasTarget) {
            currentLaunchPower = calculateLaunchPower(distance);
        } else {
            currentLaunchPower = MIN_LAUNCH_POWER;
        }

        // --- Single shot state machine ---
        if (tripleShotState == TripleShotState.IDLE) {
            switch (launchState) {
                case IDLE:
                    robot.launchMotor.setPower(currentLaunchPower);
                    if (rightTrigger > 0.1) {
                        launchState = LaunchState.FIRING;
                        launchTimer.reset();
                        robot.Trigger.setPosition(TRIGGER_FIRE_POS);
                    }
                    break;

                case FIRING:
                    if (launchTimer.seconds() >= FIRE_TIME) {
                        launchState = LaunchState.RESETTING;
                        launchTimer.reset();
                        robot.Trigger.setPosition(TRIGGER_START_POS);
                    }
                    break;

                case RESETTING:
                    if (launchTimer.seconds() >= RESET_TIME) {
                        launchState = LaunchState.IDLE;
                    }
                    break;
            }
        }

        // --- Triple shot state machine ---
        switch (tripleShotState) {
            case IDLE:
                if (tripleShot && launchState == LaunchState.IDLE && hasTarget) {
                    tripleShotState = TripleShotState.TRANSFER_RUNNING;
                    shotsFired = 0;
                    tripleShotBasePower = currentLaunchPower;
                    tripleShotTimer.reset();
                    robot.transferMotor.setPower(1.0);
                    tripleShotStarted = true;
                }
                break;

            case TRANSFER_RUNNING:
                if (tripleShotTimer.seconds() >= TRIPLE_SHOT_START_DELAY) {
                    tripleShotState = TripleShotState.FIRING_BALL;
                    tripleShotTimer.reset();
                    robot.launchMotor.setPower(tripleShotBasePower + (shotsFired * POWER_INCREASE_PER_SHOT));
                    robot.Trigger.setPosition(TRIGGER_FIRE_POS);
                }
                break;

            case FIRING_BALL:
                if (tripleShotTimer.seconds() >= FIRE_TIME) {
                    tripleShotState = TripleShotState.RESETTING_TRIGGER;
                    tripleShotTimer.reset();
                    robot.Trigger.setPosition(TRIGGER_START_POS);
                }
                break;

            case RESETTING_TRIGGER:
                if (tripleShotTimer.seconds() >= RESET_TIME) {
                    shotsFired++;
                    if (shotsFired >= TRIPLE_SHOT_COUNT) {
                        tripleShotState = TripleShotState.COMPLETE;
                        tripleShotTimer.reset();
                    } else {
                        tripleShotState = TripleShotState.WAITING_BETWEEN_SHOTS;
                        tripleShotTimer.reset();
                    }
                }
                break;

            case WAITING_BETWEEN_SHOTS:
                if (tripleShotTimer.seconds() >= TRIPLE_SHOT_DELAY) {
                    tripleShotState = TripleShotState.FIRING_BALL;
                    tripleShotTimer.reset();
                    robot.launchMotor.setPower(tripleShotBasePower + (shotsFired * POWER_INCREASE_PER_SHOT));
                    robot.Trigger.setPosition(TRIGGER_FIRE_POS);
                }
                break;

            case COMPLETE:
                robot.transferMotor.setPower(0);
                tripleShotState = TripleShotState.IDLE;
                break;
        }
    }

    public boolean isTripleShotActive() {
        return tripleShotState != TripleShotState.IDLE;
    }

    public double getCurrentLaunchPower() { return currentLaunchPower; }
    public LaunchState getLaunchState()   { return launchState; }
    public TripleShotState getTripleShotState() { return tripleShotState; }

    public double distanceCalc(double ta) {
        return 72.06169 * Math.pow(ta, -0.509117);
    }

    private double calculateLaunchPower(double distanceInches) {
        double nominalVoltage = 12.3;
        double voltageConstant = (presentVoltage >= 12 && presentVoltage <= 13)
                ? 1.0
                : nominalVoltage / presentVoltage;

        double clampedDistance  = Range.clip(distanceInches, MIN_DISTANCE, MAX_DISTANCE);
        double normalizedDistance = (clampedDistance - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE);
        double t = Math.pow(normalizedDistance, 0.6);

        return (MIN_LAUNCH_POWER + t * (MAX_LAUNCH_POWER - MIN_LAUNCH_POWER)) * voltageConstant;
    }

    private static double degToPos(double degrees) {
        // goBILDA servos typically rotate ~300 degrees over full 0–1 range
        return Range.clip(degrees / 300.0, 0.0, 1.0);
    }
}
