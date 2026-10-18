package org.firstinspires.ftc.teamcode.robot;
public class Turret {
    private RobotHardware robot;
public Turret (RobotHardware robot) {
        this.robot = robot;
}

    private boolean localized = false;
    private void aim() {
        if state != unwinding
        int transformedTargetPosition = targetHeadingToTargetPos(Math.atan2(shooterConfig.elementVelocty[0], shooterConfig.elementVelocity[1])); //smth like that
    }
    public void localizeTurret() {
        @functionalInterface
        public class HallEffectCalibrator {
            double readVoltage(double position);
        }
        public record searchBounds(double lowerBound, double upperBound) {}
    }
    public void updateTurret() {
        if (!localized) {
            localizeTurret();
            return;
        }
        emergencyUnWindIfNeeded();
        relocalizeRotationIfLimitHit();
        aim();
    }
    private void relocalizeRotationIfLimitHit() {
// if state != unwinding
        //if (limitSwitch) {
        //	reset encoder
//}
    }
    private int targetHeadingToTargetPos(double targetHeading) {
        //gear math + robot orientation and such
    }
    private double[] vectorSubtraction(double[] a, double[] b) {
        //vector subtraction logic
        //only get first two elements on list a
    }
    private void setPosition(int position, double angularVelocity) {
        //get the initial position from external encoder
        //PID here
    }


}
