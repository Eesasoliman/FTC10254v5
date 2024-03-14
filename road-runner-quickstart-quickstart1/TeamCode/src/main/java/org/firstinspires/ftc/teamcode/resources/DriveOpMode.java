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
    public static double DESIRED_DISTANCE = 6.0; //  this is how close the camera should get to the target (inches)
    final OpenCvCameraRotation CAMERA_ROTATION = OpenCvCameraRotation.UPSIDE_DOWN;
    public HardwarePushBot robot = new HardwarePushBot();
    public SampleMecanumDrive drive = null;
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    public static final double pixelHeightOffset = 0.027; // Make sure 0.35 - pixelHeightOffset * 5 + dropROffset is not less than 0.
    public static final double dropLOffset = 0.02;
    public static final double dropROffset = 0;

    public int targetDropdownHeight = 4;

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
        aprilTag.setDecimation(1);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .addProcessor(aprilTag)
                .build();
//        visionPortal.stopLiveView(); // Comment out when debugging
//        visionPortal.stopStreaming();
    }

    public void startAprilTagCamera() {
        visionPortal.resumeStreaming();
    }

    /**
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTagProcessor()
    */
    private void setManualExposure(int exposureMS, int gain) {
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

    public Pose2d relocalize(boolean isBlueSide)
    {
        int ID = (isBlueSide) ? 2 : 5;
        boolean targetFound = false;
        AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

        while (!targetFound) {
            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if (detection.id == ID) {
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

        double yaw = desiredTag.ftcPose.yaw;
        double range = desiredTag.ftcPose.range;
        double bearing = desiredTag.ftcPose.bearing;

        double tagOffsetX = range * Math.sin(Math.toRadians(yaw - bearing));
        double tagOffsetY = range * Math.cos(Math.toRadians(yaw - bearing));

        double centerOffsetX = 8.5 * Math.sin(Math.toRadians(90 - yaw));
        double centerOffsetY = 8.5 * Math.cos(Math.toRadians(90 - yaw));

        double stageX = 60 - (tagOffsetY + centerOffsetX);
        double stageY = (isBlueSide) ? (36 + tagOffsetX + centerOffsetY) : (-36 + tagOffsetX + centerOffsetY);
        double stageHeading = 0 - yaw;

//        telemetry.addData("range", range);
//        telemetry.addData("bearing", bearing);
//        telemetry.addData("yaw", yaw);
//        telemetry.addData("rX", desiredTag.ftcPose.x);
//        telemetry.addData("rY", desiredTag.ftcPose.y);
//        telemetry.addData("X", tagOffsetX);
//        telemetry.addData("Y", tagOffsetY);
//        telemetry.addData("cX", centerOffsetX);
//        telemetry.addData("cY", centerOffsetY);
//        telemetry.addData("fX", stageX);
//        telemetry.addData("fY", stageY);
//        telemetry.addData("heading", stageHeading);

        return new Pose2d(
            stageX,
            stageY,
            Math.toRadians(stageHeading)
        );
    }

    public boolean detectRobot(boolean isBlueSide) {
        int[] idArr = (isBlueSide) ? new int[]{1, 2, 3} : new int[]{4, 5, 6};
        boolean[] targetsFound = new boolean[]{false, false, false};

        double start = getRuntime();
        while (getRuntime() < start + 5 && !targetsFound[0] && !targetsFound[1] && !targetsFound[2]) {
            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            targetsFound = new boolean[]{false, false, false};
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we this is one of the target tags.
                    if (detection.id == idArr[0] || detection.id == idArr[1] || detection.id == idArr[2]) {
                        // Yes, this is one of the tags we need.
                        targetsFound[(detection.id - 1) % 3] = true;
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

        // Returns true if one of the April Tags was not found, meaning there is a robot there.
         return (!targetsFound[0] || !targetsFound[1] || !targetsFound[2]);
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
                    dropYellowPixel = true;
                    riskyWhite = false;
                }
            }

            if (gamepad1.b && gamepad1.timestamp > nextTimestamp) {
                nextTimestamp = gamepad1.timestamp + 300;
                riskyWhite = !riskyWhite;

                if (riskyWhite) {
                    dropYellowPixel = true;
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
            sleep(10);
        }

        telemetry.addLine("Initialization Complete");
        telemetry.update();
        return new boolean[]{dropYellowPixel, easyWhite, riskyWhite, parkInside};
    }

    public void setDropdown(int dropdownPos) {
        if (dropdownPos == 0) {
            robot.DROPL.setPosition(0.00 + pixelHeightOffset * 1 + dropLOffset);
            robot.DROPR.setPosition(0.35 - pixelHeightOffset * 1 + dropROffset);
        } else if (dropdownPos == 1) {
            robot.DROPL.setPosition(0.00 + pixelHeightOffset * 2 + dropLOffset);
            robot.DROPR.setPosition(0.35 - pixelHeightOffset * 2 + dropROffset);
        } else if (dropdownPos == 2) {
            robot.DROPL.setPosition(0.00 + pixelHeightOffset * 3 + dropLOffset);
            robot.DROPR.setPosition(0.35 - pixelHeightOffset * 3 + dropROffset);
        } else if (dropdownPos == 3) {
            robot.DROPL.setPosition(0.00 + pixelHeightOffset * 4 + dropLOffset);
            robot.DROPR.setPosition(0.35 - pixelHeightOffset * 4 + dropROffset);
        } else if (dropdownPos == 4) {
            robot.DROPL.setPosition(0.00 + pixelHeightOffset * 5 + dropLOffset);
            robot.DROPR.setPosition(0.35 - pixelHeightOffset * 5 + dropROffset);
        } else if (dropdownPos == 5) {
            robot.DROPL.setPosition(0.35 + dropLOffset);
            robot.DROPR.setPosition(0.00 + dropROffset);
        }
    }

    public void setTargetDropdownHeight(int targetLevel) {
        targetDropdownHeight = targetLevel;
    }

    public void prepareScoring(double moveLiftByInches)
    {
        // Swivel in
        robot.LFS.setPosition(0.95); // To swivel in more, increase this
        robot.RFS.setPosition(0.05);// To swivel in more, decrease this
        // Set CLAW to close position
        robot.CLAW.setPosition(0.5);
        // Lift upward
        lift(moveLiftByInches);
        // Keep lifting until desired height is reached or 500 milliseconds have passed
        double start = getRuntime();
        while (getRuntime() < start + 0.5 && robot.RL.isBusy()) {
            sleep(1);
        }
        // Swivel out
        robot.LFS.setPosition(0.60); // To swivel out more, decrease this
        robot.RFS.setPosition(0.40); // To swivel out more, increase this
    }

    public void scorePixelsOnBackboard(double liftEndPositionInches)
    {
        // Set CLAW to open position
        robot.CLAW.setPosition(0);
        sleep(750);
        lift(5);
    }

    public void resetForTeleOp(double dist)
    {
        robot.WRIST.setPosition(0.38); // Set WRIST to vertical position
        robot.CLAW.setPosition(0); // Set CLAW to open position
        robot.LFS.setPosition(0.95); // To swivel in more, increase this
        robot.RFS.setPosition(0.05);// To swivel in more, decrease this
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

    public void purpleIntake() {
        // Old function based on power
//        robot.IN.setPower(-.24);
//        sleep(120);
//        robot.IN.setPower(0);


        // New function based on encoder
        setDropdown(5);
        robot.IN.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.IN.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        while (robot.IN.getCurrentPosition() > -33) {
            robot.IN.setPower(-0.2);
        }
        robot.IN.setPower(0);
    }

    public void intakeTwoWhite() {
        setDropdown(targetDropdownHeight);
        robot.IN.setPower(1);
        sleep(500);
        targetDropdownHeight--;
        setDropdown(targetDropdownHeight);
        sleep(500);
        // Set CLAW to close position
        robot.CLAW.setPosition(0.5);
        setDropdown(5);
        robot.IN.setPower(-1);
        sleep(150);
        robot.IN.setPower(0);
    }

    public void intakeOneWhite() {
        setDropdown(targetDropdownHeight);
        robot.IN.setPower(1);
        sleep(500);
        // Set CLAW to close position
        robot.CLAW.setPosition(0.5);
        setDropdown(5);
        robot.IN.setPower(-1);
        sleep(150);
        robot.IN.setPower(0);
    }
}
