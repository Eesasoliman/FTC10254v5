package org.firstinspires.ftc.teamcode.resources;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.HardwarePushBot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * This class was created to host all sizeable functions for use in actual op modes. In other words, this is an FTC library for real autonomous and/or teleop programs.
 */

// https://www.reddit.com/r/FTC/comments/auczw5/roadrunner_help/
@Disabled
@Autonomous
public class DriveOpMode extends LinearOpMode {
    final OpenCvCameraRotation CAMERA_ROTATION = OpenCvCameraRotation.UPSIDE_DOWN;
    public HardwarePushBot robot = new HardwarePushBot();
    public SampleMecanumDrive drive = null;

    final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)
    private final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    /**
     * LinearOpMode requires a runOpMode function, but this method should be overridden in all other scripts that use DriveOpMode.
     */
    @Override
    public void runOpMode() {}

    /**
     * DriveOpMode requires this function to be called only once if the function getTrajectories() will be used at some point in the script.
     */
    public SampleMecanumDrive initDriveOpMode()
    {
        robot.init(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        return drive;
    }

    /**
     * To access the pipeline returned from this function, use: BlueColorPipeline pipeline = startBlueCamera();
     */
    public BlueColorPipeline startBlueCamera()
    {
        BlueColorPipeline pipeline = new BlueColorPipeline();
        OpenCvWebcam CAM;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        CAM = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        CAM.setMillisecondsPermissionTimeout(2500);

        CAM.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                CAM.setPipeline(pipeline);
                CAM.startStreaming(1280, 720, CAMERA_ROTATION);
            }

            @Override
            public void onError(int errorCode) {}
        });

        return pipeline;
    }

    /**
     * To access the pipeline returned from this function, use: RedColorPipeline pipeline = startRedCamera();
     */
    public RedColorPipeline startRedCamera()
    {
        RedColorPipeline pipeline = new RedColorPipeline();
        OpenCvWebcam CAM;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        CAM = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        CAM.setMillisecondsPermissionTimeout(2500);

        CAM.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                CAM.setPipeline(pipeline);
                CAM.startStreaming(1280, 720, CAMERA_ROTATION);
            }

            @Override
            public void onError(int errorCode) {}
        });

        return pipeline;
    }

    /**
     * Initialize the AprilTag processor.
     */
    public void initAprilTagProcessor() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .addProcessor(aprilTag)
                .build();
        visionPortal.stopLiveView(); // Comment out when debugging
        visionPortal.stopStreaming();
    }

    public void startAprilTagCamera() {
        visionPortal.resumeStreaming();
    }

    /**
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTagProcessor()
    */
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    public TrajectorySequence relocalize(Pose2d startPose, double lateralOffset)
    {
        boolean targetFound = false;
        desiredTag = null;

        while (!targetFound) {
            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }
            sleep(10);
        }

        visionPortal.stopStreaming();

        double relativeToTagXError = desiredTag.ftcPose.x;
        double relativeToTagYError = desiredTag.ftcPose.y;
        double relativeToTagHeadingError = desiredTag.ftcPose.bearing;

        Pose2d endPose = new Pose2d(
            startPose.getX() + relativeToTagYError - DESIRED_DISTANCE,
            startPose.getY() + relativeToTagXError + lateralOffset,
            startPose.getHeading() + Math.toRadians(relativeToTagHeadingError));
        return drive.trajectorySequenceBuilder(startPose).lineToLinearHeading(endPose).build();
    }

    public boolean[] initWithController(boolean park)
    {
        long nextTimestamp = 0;
        boolean dropYellowPixel = true;
        boolean easyWhite = true;
        boolean riskyWhite = false;
        boolean parkInside = park;

        while (!gamepad1.right_bumper)
        {
            if (gamepad1.x && gamepad1.timestamp > nextTimestamp) {
                nextTimestamp = gamepad1.timestamp + 300;
                dropYellowPixel = !dropYellowPixel;
            }

            if (gamepad1.a && gamepad1.timestamp > nextTimestamp) {
                nextTimestamp = gamepad1.timestamp + 300;
                easyWhite = !easyWhite;

                if (easyWhite)
                {
                    dropYellowPixel = false;
                }
            }

            if (gamepad1.b && gamepad1.timestamp > nextTimestamp) {
                nextTimestamp = gamepad1.timestamp + 300;
                riskyWhite = !riskyWhite;

                if (riskyWhite) {
                    dropYellowPixel = false;
                    easyWhite = false;
                }
            }

            if (gamepad1.y && gamepad1.timestamp > nextTimestamp) {
                nextTimestamp = gamepad1.timestamp + 300;
                parkInside = !parkInside;
            }

            // Press right bumper when you are done initializing.
            telemetry.addData("2+0 (x)", dropYellowPixel);
            telemetry.addData("CLOSE 2+1 | TRUSS 2+2 (a)", easyWhite);
            telemetry.addData("CLOSE 2+3 | TRUSS 2+4 (b)", riskyWhite);
            telemetry.addData("parkInside (y)", parkInside);
            telemetry.update();
        }

        return new boolean[]{dropYellowPixel, easyWhite, riskyWhite, parkInside};
    }

    public void setDropdownLevel(int level)
    {
        if (level == 0) {
            robot.DROPDOWN.setPosition(0.00);
        } else if (level == 1) {
            robot.DROPDOWN.setPosition(0.25);
        } else if (level == 2) {
            robot.DROPDOWN.setPosition(0.50);
        } else if (level == 3) {
            robot.DROPDOWN.setPosition(0.75);
        } else if (level == 4) {
            robot.DROPDOWN.setPosition(1.00);
        }
    }

    public void prepareScoring(double moveLiftByInches)
    {
        // Swivel in
        robot.LFS.setPosition(0.95);
        robot.RFS.setPosition(0.05);
        // Lock carriage servos
//        robot.BPS.setPosition(.4);
//        robot.FPS.setPosition(0);
        // Lift upward
        lift(moveLiftByInches);
        sleep(500);
        // Swivel out
        robot.LFS.setPosition(0.45); //lower to make it go more out
        robot.RFS.setPosition(0.55); //higher to make it go more out
    }

    public void scorePixelsOnBackboard(double liftEndPositionInches)
    {
//        robot.BPS.setPosition(0.40);
//        robot.FPS.setPosition(0.40);
        sleep(750);
        lift(5);
    }

    public void resetForTeleOp(double dist)
    {
        robot.LFS.setPosition(0.95);
        robot.RFS.setPosition(0.05);
        lift(-dist-4);
    }


    public void lift(double inches) {
        robot.LL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.RL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.LL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.RL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        int dir = -1;
        double cpi = 83.34;

        if (inches > 0){
            dir = 1;
        }

        robot.LL.setTargetPosition((int) (-1 * inches * cpi));
        robot.RL.setTargetPosition((int) (-1 * inches * cpi));

        robot.LL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.LL.setPower(0.7 * dir);
        robot.RL.setPower(0.7 * dir);
    }

    public void intake() {
//        robot.IN.setPower(-.24);
//        sleep(120);
//        robot.IN.setPower(0);

        robot.IN.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        robot.IN.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        while (robot.IN.getCurrentPosition() > -33) {
            robot.IN.setPower(-0.2);
        }
        robot.IN.setPower(0);
    }
}
