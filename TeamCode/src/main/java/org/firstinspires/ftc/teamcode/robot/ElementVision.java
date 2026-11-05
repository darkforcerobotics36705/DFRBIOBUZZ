
package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.RotatedRect;
import android.util.Size;
import java.util.Arrays;
import java.util.List;

public class ElementVision {

    private static final int IMAGE_WIDTH = 1280;
    private static final double HORIZONTAL_FOV_DEGREES = 48.8;

    private final ColorBlobLocatorProcessor blueProcessor;
    private final ColorBlobLocatorProcessor redProcessor;
    private final ColorBlobLocatorProcessor yellowProcessor;

    private final List<ColorBlobLocatorProcessor> colorProcessors;

    private VisionPortal visionPortal;

    private boolean autoFinding = false;
    private boolean targetVisible = false;

    private double elementAngleDegrees = Double.NaN;
    private double elementDistanceInches = Double.NaN;

    public ElementVision(HardwareMap hardwareMap) {

        // Create a processor for each color.
        blueProcessor = createProcessor(ColorRange.BLUE);
        redProcessor = createProcessor(ColorRange.RED);
        yellowProcessor = createProcessor(ColorRange.YELLOW);

        colorProcessors = Arrays.asList(
                blueProcessor,
                redProcessor,
                yellowProcessor
        );

        // Get the Logitech webcam using its Robot Configuration name.
        WebcamName webcam = hardwareMap.get(
                WebcamName.class,
                "logitech"
        );

        // Create the VisionPortal and attach all three processors.
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new Size(IMAGE_WIDTH, 720))
                .addProcessor(blueProcessor)
                .addProcessor(redProcessor)
                .addProcessor(yellowProcessor)
                .build();
    }

    private ColorBlobLocatorProcessor createProcessor(
            ColorRange colorRange) {

        return new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(colorRange)
                .setContourMode(
                        ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY
                )
                .setBlurSize(5)
                .build();
    }

    public void beginAutoFind() {
        autoFinding = true;
        targetVisible = false;
        elementAngleDegrees = Double.NaN;
        elementDistanceInches = Double.NaN;
    }
    //
    // Call repeatedly while looking for a ball.
    public void update() {
        targetVisible = false;
        elementAngleDegrees = Double.NaN;
        elementDistanceInches = Double.NaN;

        if (!autoFinding) {
            return;
        }

        ColorBlobLocatorProcessor.Blob largestBlob = null;

        // Compare blobs across all three color processors.
        for (ColorBlobLocatorProcessor processor : colorProcessors) {
            List<ColorBlobLocatorProcessor.Blob> blobs =
                    processor.getBlobs();

            for (ColorBlobLocatorProcessor.Blob blob : blobs) {
                if (largestBlob == null
                        || blob.getContourArea()
                        > largestBlob.getContourArea()) {
                    largestBlob = blob;
                }
            }
        }

        if (largestBlob == null) {
            return;
        }

        RotatedRect box = largestBlob.getBoxFit();
        double centerX = box.center.x;

        double focalLengthPixels =
                (IMAGE_WIDTH / 2.0)
                        / Math.tan(
                        Math.toRadians(HORIZONTAL_FOV_DEGREES / 2.0)
                );

        elementAngleDegrees = Math.toDegrees(
                Math.atan(
                        (centerX - IMAGE_WIDTH / 2.0)
                                / focalLengthPixels
                )
        );

        // Ball sizes vary, so distance cannot be determined
        // reliably from blob size alone.
        elementDistanceInches = Double.NaN;

        targetVisible = true;
    }

    public boolean elementDetected() {
        return autoFinding && targetVisible;
    }

    public double getElementDeltaX() {
        return elementDetected()
                ? elementAngleDegrees
                : Double.NaN;
    }

    public double getElementDistanceInches() {
        return Double.NaN;
    }

    public void stopAutoFind() {
        autoFinding = false;
        targetVisible = false;
        elementAngleDegrees = Double.NaN;
        elementDistanceInches = Double.NaN;
    }

    // Call when the OpMode is finished with this camera.
    public void close() {
        autoFinding = false;
        targetVisible = false;

        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
    }
}
