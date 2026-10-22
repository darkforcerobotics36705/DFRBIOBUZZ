package org.firstinspires.ftc.teamcode.robot;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Locale;
import java.util.Queue;

/**
 * Sensor-only storage tracker for two lanes, four elements total.
 * Per lane: stacked BB1+BB2 at intake, one color sensor just after them,
 * and BB3 near the turret/shooter exit.
 *
 * Storage never sets motor power or LED output. Call checkBeamBreaks()
 * once per robot loop and pass the ACTUAL intake direction.
 */
public class Storage {
    public enum Lane { LEFT, RIGHT }
    public enum IntakeState { FORWARD, REVERSE, STOPPED }

    /** Connect these calls to the actual RobotHardware sensor fields. */
    public interface Hardware {
        boolean beamBroken(Lane lane, int beam); // 1, 2, or 3
        String readColor(Lane lane);             // RED, BLUE, YELLOW, UNKNOWN
    }

    private static final int CAPACITY = 4;
    private static final long COLOR_GRACE_NS = 150_000_000L;

    private static final class StoredElement {
        final Lane lane;
        final String color;
        StoredElement(Lane lane, String color) {
            this.lane = lane;
            this.color = color;
        }
    }

    private static final class LaneState {
        boolean previousEntrance;
        boolean previousExit;
        boolean awaitingColor;
        long colorDeadlineNs;
        boolean rejected;
        // During a reverse passage, a previously stored element can be removed
        // only when its color is confirmed and the entrance subsequently clears.
        boolean reversePassage;
        boolean reverseMatchedStoredElement;
        // An element rejected before counting must never remove a stored item.
        boolean rejectedPassage;
    }

    private final Hardware robot;
    private final LaneState leftState = new LaneState();
    private final LaneState rightState = new LaneState();
    private final Deque<StoredElement> leftQueue = new ArrayDeque<>();
    private final Deque<StoredElement> rightQueue = new ArrayDeque<>();
    private final Deque<StoredElement> arrivalOrder = new ArrayDeque<>();
    private boolean emergencyOuttakeNeeded;

    public Storage(Hardware robot) {
        if (robot == null) throw new IllegalArgumentException("Hardware cannot be null");
        this.robot = robot;
    }

    /**
     * UPDATE: Original required signature. Assumes forward movement.
     * Use the IntakeState overload whenever the intake can reverse or stop.
     */
    public boolean checkBeamBreaks(String allianceColor) {
        return checkBeamBreaks(allianceColor, IntakeState.FORWARD);
    }

    /**
     * UPDATE: Backward-compatible overload. false means STOPPED/unknown,
     * NOT reverse; use the IntakeState overload for reverse-exit detection.
     */
    public boolean checkBeamBreaks(String allianceColor, boolean forwardExitEnabled) {
        return checkBeamBreaks(allianceColor,
                forwardExitEnabled ? IntakeState.FORWARD : IntakeState.STOPPED);
    }

    /**
     * UPDATE: Reads sensors, tracks incoming colors, and detects exits.
     * Returns true when the main control code should reject an element.
     */
    public boolean checkBeamBreaks(String allianceColor, IntakeState intakeState) {
        String alliance = normalize(allianceColor);
        if (!"RED".equals(alliance) && !"BLUE".equals(alliance)) {
            throw new IllegalArgumentException("Alliance must be RED or BLUE");
        }
        if (intakeState == null) throw new IllegalArgumentException("Intake state cannot be null");
        long now = System.nanoTime();
        processLane(Lane.LEFT, leftState, alliance, intakeState, now);
        processLane(Lane.RIGHT, rightState, alliance, intakeState, now);
        emergencyOuttakeNeeded = leftState.rejected || rightState.rejected;
        return emergencyOuttakeNeeded;
    }

    /** UPDATE: Processes one lane without commanding any other subsystem. */
    private void processLane(Lane lane, LaneState state, String alliance,
                             IntakeState direction, long now) {
        boolean entrance = robot.beamBroken(lane, 1) || robot.beamBroken(lane, 2);
        boolean exit = robot.beamBroken(lane, 3);
        boolean entranceRising = entrance && !state.previousEntrance;
        boolean entranceFalling = !entrance && state.previousEntrance;

        if (direction == IntakeState.FORWARD) {
            state.reversePassage = false;
            state.reverseMatchedStoredElement = false;
            state.rejectedPassage = false;

            if (entranceRising && !state.rejected) {
                state.awaitingColor = true;
                state.colorDeadlineNs = now + COLOR_GRACE_NS;
            }
            if (entrance && state.awaitingColor) {
                state.colorDeadlineNs = now + COLOR_GRACE_NS;
            }

            // Color sensor is read only for an entrance-triggered candidate,
            // or to verify that a previously rejected element has cleared.
            if (state.awaitingColor) {
                String color = normalize(robot.readColor(lane));
                if (!"UNKNOWN".equals(color)) {
                    state.awaitingColor = false;
                    if (!isAllowed(color, alliance)) {
                        state.rejected = true;
                    } else if (!isFull()) {
                        StoredElement element = new StoredElement(lane, color);
                        queueFor(lane).addLast(element);
                        arrivalOrder.addLast(element);
                    }
                } else if (!entrance && now >= state.colorDeadlineNs) {
                    state.awaitingColor = false;
                    state.rejected = true;
                }
            }

            // Do not clear rejection just because a sensor temporarily reads
            // UNKNOWN. Rejection is cleared after a reverse exit is observed.
            if (state.previousExit && !exit) {
                removeForwardExit(lane);
            }
        } else if (direction == IntakeState.REVERSE) {
            // No new elements are added while reversing.
            state.awaitingColor = false;
            if (entranceRising) {
                state.reversePassage = true;
                state.rejectedPassage = state.rejected;
                state.reverseMatchedStoredElement = false;
            }

            if (state.reversePassage && entrance && !state.rejectedPassage) {
                // Read only while entrance beams are blocked. If the color
                // matches the newest stored element, it may be that element
                // traveling backward, rather than an uncounted rejected ball.
                StoredElement newest = queueFor(lane).peekLast();
                if (newest != null) {
                    String color = normalize(robot.readColor(lane));
                    if (newest.color.equals(color)) {
                        state.reverseMatchedStoredElement = true;
                    }
                }
            }

            if (entranceFalling && state.reversePassage) {
                if (state.rejectedPassage) {
                    // Rejected balls were never counted, so do not dequeue.
                    state.rejected = false;
                } else if (state.reverseMatchedStoredElement) {
                    removeReverseExit(lane);
                }
                state.reversePassage = false;
                state.reverseMatchedStoredElement = false;
                state.rejectedPassage = false;
            }
            // If a rejection began after the entrance was already blocked,
            // recognize the existing blocked interval as its reverse passage.
            if (entrance && state.rejected && !state.reversePassage) {
                state.reversePassage = true;
                state.rejectedPassage = true;
            }
        } else {
            // STOPPED: do not infer movement or remove anything.
            state.reversePassage = false;
            state.reverseMatchedStoredElement = false;
            state.rejectedPassage = false;
        }

        state.previousEntrance = entrance;
        state.previousExit = exit;
    }

    /** STATUS: True if any entrance beam (BB1/BB2) is blocked. */
    public boolean anyIntakeBeamBroken() {
        return robot.beamBroken(Lane.LEFT, 1) || robot.beamBroken(Lane.LEFT, 2)
                || robot.beamBroken(Lane.RIGHT, 1) || robot.beamBroken(Lane.RIGHT, 2);
    }

    /** STATUS: Copy of all stored colors in chronological arrival order. */
    public Queue<String> getElementQueue() {
        Queue<String> result = new ArrayDeque<>();
        for (StoredElement element : arrivalOrder) result.add(element.color);
        return result;
    }

    /** STATUS: Copy of one lane's colors, oldest first. */
    public Queue<String> getElementQueue(Lane lane) {
        Queue<String> result = new ArrayDeque<>();
        for (StoredElement element : queueFor(lane)) result.add(element.color);
        return result;
    }

    public int getStoredElements() { return arrivalOrder.size(); }
    public int getCount(Lane lane) { return queueFor(lane).size(); }
    public boolean isFull() { return getStoredElements() >= CAPACITY; }
    public boolean needsEmergencyOuttake() { return emergencyOuttakeNeeded; }

    /** STATUS: One goBILDA RGB light, showing the combined count. */
    public String getStorageLightColor() {
        switch (getStoredElements()) {
            case 0: return "VIOLET";
            case 1: return "BLUE";
            case 2: return "GREEN";
            case 3: return "YELLOW";
            default: return "RED";
        }
    }

    /** UPDATE: Only call if a forward exit is confirmed independently of BB3. */
    public boolean recordForwardExit(Lane lane) { return removeForwardExit(lane); }

    private boolean removeForwardExit(Lane lane) {
        StoredElement removed = queueFor(lane).pollFirst();
        if (removed == null) return false;
        arrivalOrder.remove(removed);
        return true;
    }

    /** UPDATE: Manual confirmation of a backward exit (do not double-count). */
    public boolean recordReverseExit(Lane lane) { return removeReverseExit(lane); }

    private boolean removeReverseExit(Lane lane) {
        StoredElement removed = queueFor(lane).pollLast();
        if (removed == null) return false;
        arrivalOrder.remove(removed);
        return true;
    }

    /** UPDATE: Clear queues and sensor-tracking state. */
    public void reset() {
        leftQueue.clear();
        rightQueue.clear();
        arrivalOrder.clear();
        resetLane(leftState);
        resetLane(rightState);
        emergencyOuttakeNeeded = false;
    }

    private static void resetLane(LaneState state) {
        state.previousEntrance = false;
        state.previousExit = false;
        state.awaitingColor = false;
        state.colorDeadlineNs = 0L;
        state.rejected = false;
        state.reversePassage = false;
        state.reverseMatchedStoredElement = false;
        state.rejectedPassage = false;
    }

    private Deque<StoredElement> queueFor(Lane lane) {
        if (lane == Lane.LEFT) return leftQueue;
        if (lane == Lane.RIGHT) return rightQueue;
        throw new IllegalArgumentException("Lane must be LEFT or RIGHT");
    }

    private static boolean isAllowed(String color, String alliance) {
        return "YELLOW".equals(color) || alliance.equals(color);
    }

    private static String normalize(String value) {
        return value == null ? "UNKNOWN" : value.trim().toUpperCase(Locale.US);
    }
}
