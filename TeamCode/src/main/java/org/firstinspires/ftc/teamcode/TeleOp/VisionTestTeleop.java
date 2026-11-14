package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpenCV.OpenCVComponent;
import org.firstinspires.ftc.teamcode.OpenCV.SamplePipeline;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

import java.util.List;

/**
 * Camera-only smoke test — no motors, servos or sensors required.
 *
 * <p>Use this on the bench before running a competition TeleOp:
 * <ul>
 *   <li>confirms the webcam opens and streams (name, resolution, FPS),</li>
 *   <li>cycles all four pipelines: A = RED, B = BLUE, X = YELLOW,
 *       Y = GOAL,</li>
 *   <li>shows pixel width / centroid for focal-length calibration,</li>
 *   <li>shows the exposure/gain/white-balance state actually applied.</li>
 * </ul>
 *
 * <p>If this OpMode streams, the vision path in
 * {@link GenericTeleop} will stream too — anything else that fails is a
 * robot/hardware issue, not the camera.
 */
@TeleOp(name = "Vision Test", group = "Test")
public class VisionTestTeleop extends OpMode {

    private OpenCVComponent vision;

    private boolean prevA;
    private boolean prevB;
    private boolean prevX;
    private boolean prevY;

    @Override
    public void init() {
        vision = new OpenCVComponent(hardwareMap, RobotConfig.Hardware.WEBCAM_NAME,
                telemetry, OpenCVComponent.Target.RED);
        vision.startStreaming();
        telemetry.addLine("VISION TEST — no other hardware needed");
        telemetry.addLine("A=RED  B=BLUE  X=YELLOW  Y=GOAL");
        telemetry.addLine("Camera: " + vision.getCameraStatus());
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;
        if (a && !prevA) {
            vision.setTarget(OpenCVComponent.Target.RED);
        }
        if (b && !prevB) {
            vision.setTarget(OpenCVComponent.Target.BLUE);
        }
        if (x && !prevX) {
            vision.setTarget(OpenCVComponent.Target.YELLOW);
        }
        if (y && !prevY) {
            vision.setTarget(OpenCVComponent.Target.GOAL);
        }
        prevA = a;
        prevB = b;
        prevX = x;
        prevY = y;

        telemetry.addLine("A=RED  B=BLUE  X=YELLOW  Y=GOAL");
        vision.reportTelemetry();

        // Calibration block: focal px = distance_in * box_width_px / known_width_in.
        List<SamplePipeline.Detection> dets = vision.getDetections();
        if (!dets.isEmpty()) {
            SamplePipeline.Detection d = dets.get(0);
            int frameWidth = vision.getFrameWidth();
            telemetry.addData("Calibration",
                    "frame %dx%d, box %dpx, focal %.1fpx, HFOV %.1f deg",
                    frameWidth, vision.getFrameHeight(), d.widthPx,
                    SamplePipeline.focalLengthPx(frameWidth),
                    RobotConfig.Vision.OPENCV_HFOV_DEG);
            telemetry.addData("Calibration hint",
                    "measure D(in) and box W(px): OPENCV_FOCAL_PX = D*W/knownWidth");
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        if (vision != null) {
            vision.close();
        }
    }
}
