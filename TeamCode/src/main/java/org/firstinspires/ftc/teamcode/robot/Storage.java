package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Locale;
import java.util.Queue;

/** Tracks up to four elements in two lanes. Only directly controls the RGB light. */
public class Storage {
    public enum Lane { LEFT, RIGHT }
    public enum IntakeState { FORWARD, REVERSE, STOPPED }
    private static final int CAPACITY = 4;
    private static final long COLOR_WAIT_NS = 150_000_000L; // 0.15 seconds
    private static final double DETECT_DISTANCE_CM = 2.5;  // Calibrate on robot!
    private static class Element {
        final Lane lane;
        final String color;
        Element(Lane lane, String color) { this.lane = lane; this.color = color; }
    }

    private static class LaneState {
        boolean previousEntrance, previousExit;
        boolean waitingForColor, reject;
        boolean reversePassage, rejectedPassage, reverseColorMatches;
        long colorDeadline;
    }

    private final RobotHardware robot;
    private final LaneState leftState = new LaneState();
    private final LaneState rightState = new LaneState();
    private final Deque<Element> leftQueue = new ArrayDeque<>();
    private final Deque<Element> rightQueue = new ArrayDeque<>();
    private final Deque<Element> arrivalOrder = new ArrayDeque<>();
    private boolean emergencyOuttakeNeeded;
    private int lastLightCount = -1;

    public Storage(RobotHardware robot) {
        if (robot == null) throw new IllegalArgumentException("RobotHardware cannot be null");
        this.robot = robot;
        updateStorageLight(); // Violet: empty storage
    }

    // ------ Main update: call this EVERY robot loop ------

    /** Required method. Use the overload with direction when intake may reverse. */
    public boolean checkBeamBreaks(String allianceColor) {
        return checkBeamBreaks(allianceColor, IntakeState.FORWARD);
    }

    /** True means the MAIN CODE should command emergency outtake. */
    public boolean checkBeamBreaks(String allianceColor, IntakeState direction) {
        String alliance = allianceColor == null ? "" : allianceColor.trim().toUpperCase(Locale.US);
        if (!alliance.equals("RED") && !alliance.equals("BLUE"))
            throw new IllegalArgumentException("Alliance must be RED or BLUE");
        if (direction == null) throw new IllegalArgumentException("Intake direction is required");

        long now = System.nanoTime();
        updateLane(Lane.LEFT, leftState, alliance, direction, now);
        updateLane(Lane.RIGHT, rightState, alliance, direction, now);
        emergencyOuttakeNeeded = leftState.reject || rightState.reject;
        updateStorageLight();
        return emergencyOuttakeNeeded;
    }

    private void updateLane(Lane lane, LaneState state, String alliance,
                            IntakeState direction, long now) {
        // Entrance beams are stacked and count as ONE detection location.
        boolean entrance = beamBroken(lane, 1) || beamBroken(lane, 2);
        boolean exit = beamBroken(lane, 3); // Near the turret
        boolean justEntered = entrance && !state.previousEntrance;
        boolean justClearedEntrance = !entrance && state.previousEntrance;
        boolean justClearedExit = !exit && state.previousExit;

        if (direction == IntakeState.FORWARD) {
            state.reversePassage = false;
            if (justEntered && !state.reject) state.waitingForColor = true;
            if (entrance && state.waitingForColor)
                state.colorDeadline = now + COLOR_WAIT_NS;

            // Only read the REV color sensor after beam-break detection.
            if (state.waitingForColor) {
                String color = readColor(lane);
                if (!color.equals("UNKNOWN")) {
                    state.waitingForColor = false;
                    if (!color.equals("YELLOW") && !color.equals(alliance)) {
                        state.reject = true;
                    } else if (!isFull()) {
                        Element element = new Element(lane, color);
                        laneQueue(lane).addLast(element);
                        arrivalOrder.addLast(element);
                    }
                } else if (!entrance && now >= state.colorDeadline) {
                    state.waitingForColor = false;
                    state.reject = true; // Element arrived but could not be identified
                }
            }
            if (justClearedExit) removeElement(lane, false);
        } else if (direction == IntakeState.REVERSE) {
            state.waitingForColor = false;
            if (justEntered || (entrance && state.reject && !state.reversePassage)) {
                state.reversePassage = true;
                state.rejectedPassage = state.reject;
                state.reverseColorMatches = false;
            }
            // Heuristic: color match + entrance clearing suggests a stored ball went out.
            if (state.reversePassage && entrance && !state.rejectedPassage) {
                Element newest = laneQueue(lane).peekLast();
                if (newest != null && newest.color.equals(readColor(lane)))
                    state.reverseColorMatches = true;
            }
            if (justClearedEntrance && state.reversePassage) {
                if (state.rejectedPassage) state.reject = false;
                else if (state.reverseColorMatches) removeElement(lane, true);
                state.reversePassage = false;
            }
        } else { // STOPPED
            state.reversePassage = false;
        }

        state.previousEntrance = entrance;
        state.previousExit = exit;
    }

    // ------ Physical sensor readings ------

    private boolean beamBroken(Lane lane, int number) {
        DigitalChannel sensor;
        if (lane == Lane.LEFT) {
            sensor = number == 1 ? robot.leftBeam1 :
                    number == 2 ? robot.leftBeam2 : robot.leftBeam3;
        } else {
            sensor = number == 1 ? robot.rightBeam1 :
                    number == 2 ? robot.rightBeam2 : robot.rightBeam3;
        }
        // Adafruit 2167: LOW means broken, with appropriate 3.3V pull-up.
        return !sensor.getState();
    }

    /** UNKNOWN means no close element or an unrecognized color. Calibrate thresholds. */
    private String readColor(Lane lane) {
        RevColorSensorV3 sensor = lane == Lane.LEFT ? robot.leftColor : robot.rightColor;
        if (sensor.getDistance(DistanceUnit.CM) > DETECT_DISTANCE_CM) return "UNKNOWN";

        double red = sensor.red(), green = sensor.green(), blue = sensor.blue();
        if (red < 1 && green < 1 && blue < 1) return "UNKNOWN";
        if (red > green * 1.4 && red > blue * 1.4) return "RED";
        if (blue > red * 1.25 && blue > green * 1.15) return "BLUE";
        if (red > blue * 1.3 && green > blue * 1.25 && red > green * 0.7)
            return "YELLOW";
        return "UNKNOWN";
    }

    // ------ Required and additional status methods ------

    public boolean anyIntakeBeamBroken() {
        return beamBroken(Lane.LEFT, 1) || beamBroken(Lane.LEFT, 2)
                || beamBroken(Lane.RIGHT, 1) || beamBroken(Lane.RIGHT, 2);
    }

    public Queue<String> getElementQueue() { return colorsFrom(arrivalOrder); }
    public Queue<String> getElementQueue(Lane lane) { return colorsFrom(laneQueue(lane)); }

    private Queue<String> colorsFrom(Deque<Element> source) {
        Queue<String> result = new ArrayDeque<>();
        for (Element element : source) result.add(element.color);
        return result;
    }

    public int getStoredElements() { return arrivalOrder.size(); }
    public int getCount(Lane lane) { return laneQueue(lane).size(); }
    public boolean isFull() { return getStoredElements() >= CAPACITY; }
    public boolean needsEmergencyOuttake() { return emergencyOuttakeNeeded; }

    /** Actually updates the single goBILDA RGB indicator via RobotHardware. */
    public void updateStorageLight() {
        int count = getStoredElements();
        if (count == lastLightCount) return;
        double position;
        switch (count) {
            case 0: position = 0.700; break; // Violet
            case 1: position = 0.611; break; // Blue
            case 2: position = 0.500; break; // Green
            case 3: position = 0.388; break; // Yellow
            default: position = 0.280; break; // Red, at 4
        }
        robot.rgbIndicator.setPosition(position);
        lastLightCount = count;
    }

    // ------ Remove elements / reset ------

    /** Only call for exits NOT already counted automatically at BB3. */
    public boolean recordForwardExit(Lane lane) { return removeElement(lane, false); }
    public boolean recordReverseExit(Lane lane) { return removeElement(lane, true); }

    private boolean removeElement(Lane lane, boolean reverse) {
        Deque<Element> queue = laneQueue(lane);
        Element removed = reverse ? queue.pollLast() : queue.pollFirst();
        if (removed == null) return false;
        arrivalOrder.remove(removed);
        updateStorageLight();
        return true;
    }

    public void reset() {
        leftQueue.clear(); rightQueue.clear(); arrivalOrder.clear();
        resetLane(leftState); resetLane(rightState);
        emergencyOuttakeNeeded = false;
        lastLightCount = -1;
        updateStorageLight();
    }

    private static void resetLane(LaneState state) {
        state.previousEntrance = state.previousExit = false;
        state.waitingForColor = state.reject = false;
        state.reversePassage = state.rejectedPassage = state.reverseColorMatches = false;
        state.colorDeadline = 0;
    }

    private Deque<Element> laneQueue(Lane lane) {
        if (lane == Lane.LEFT) return leftQueue;
        if (lane == Lane.RIGHT) return rightQueue;
        throw new IllegalArgumentException("Lane must be LEFT or RIGHT");
    }
}
