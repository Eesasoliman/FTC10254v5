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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.opencv.core.Scalar;
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
    private OpenCvWebcam CAM;
    final OpenCvCameraRotation CAMERA_ROTATION = OpenCvCameraRotation.UPSIDE_DOWN;
    public HardwarePushBot robot = new HardwarePushBot();
    public SampleMecanumDrive drive = null;
    private VisionPortal visionPortal; // Used to manage the video source.
    private AprilTagProcessor aprilTag; // Used for managing the AprilTag detection process.
    public static final double pixelHeightOffset = 0.02; // Make sure 0.35 - pixelHeightOffset * 5 + dropROffset is not less than 0.
    public static final double dropLOffset = 0.026;
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
     * To access the pipeline returned from this function, use: ColorPipeline pipeline = startCameras(true/false);
     * @param isBlueSide True if the pipeline looks for the color blue. False if it looks for the color red.
     */
    public ColorPipeline startCameras(boolean isBlueSide)
    {
        ColorPipeline pipeline = (isBlueSide) ?
                (new ColorPipeline( // Blue Lower and Upper Boundaries
                        new Scalar(179 * (200/360d), 255 * (70/100d), 255*(20/100d)),
                        new Scalar(179 * (240/360d), 255 * (100/100d), 255*(70/100d)))):
                (new ColorPipeline( // Red Lower and Upper Boundaries
//                        new Scalar(179 * (0/360d), 255 * (85/100d), 255*(30/100d)),
                        new Scalar(179 * (0/360d), 255 * (50/100d), 255*(30/100d)),
                        new Scalar(179 * (355/360d), 255 * (95/100d), 255*(95/100d))));

        // Enable live view for this webcam
//        CAM = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"),  hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
        // Disable live view for this webcam
        CAM = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
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
         visionPortal.stopLiveView(); // Comment out when you need the april tag camera stream to debug something or when live view is off for this camera
        visionPortal.stopStreaming();

        return pipeline;
    }

    public void closeColorPipelineCamera() {
        hardwareMap.get(WebcamName.class, "Webcam 1").close();
        CAM.closeCameraDevice();
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

    public void resumeStreaming() {
        visionPortal.resumeStreaming();
    }

    public void waitForAprilTagCamera() {
//        if (visionPortal.getCameraState() == VisionPortal.CameraState.CAMERA_DEVICE_CLOSED || visionPortal.getCameraState() == VisionPortal.CameraState.CLOSING_CAMERA_DEVICE || visionPortal.getCameraState() == VisionPortal.CameraState.ERROR) {
//            aprilTag = null;
//            visionPortal = null;
//            initAprilTagProcessor();
//            while (visionPortal.getCameraState() != VisionPortal.CameraState.CAMERA_DEVICE_READY) {
//                sleep(10);
//            }
//        }
        if (visionPortal.getCameraState() == VisionPortal.CameraState.CAMERA_DEVICE_READY) {
            visionPortal.resumeStreaming();
        }
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addLine("Waiting for camera to start streaming...");
            telemetry.update();
            while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                sleep(5);
            }
        }
    }

    public Pose2d relocalize(boolean isBlueSide)
    {
//        waitForAprilTagCamera();

        int ID = (isBlueSide) ? 2 : 5;
        boolean targetFound = false;
        AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

        while (!targetFound) {
            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getFreshDetections();
            if (currentDetections == null) {
                continue;
            }
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if (detection.id == ID) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    }
                }
            }
            sleep(10);
        }

//        visionPortal.stopStreaming();

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
        telemetry.addData("fX", stageX);
        telemetry.addData("fY", stageY);
        telemetry.addData("heading", stageHeading);
        telemetry.update();

        return new Pose2d(
            stageX,
            stageY,
            Math.toRadians(stageHeading)
        );
    }

    /**
     *
     * @return True if one of the april tags weren't detected, meaning there is a robot there. False if all april tags were detected.
     */
    public boolean detectRobot(boolean isBlueSide) {
        waitForAprilTagCamera();
        telemetry.setAutoClear(false);

        int[] idArr = (isBlueSide) ? new int[]{1, 2, 3} : new int[]{4, 5, 6};

        double start = getRuntime();
        while (getRuntime() < start + 5) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            boolean[] targetsFound = new boolean[]{false, false, false};
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    if (detection.id == idArr[0] || detection.id == idArr[1] || detection.id == idArr[2]) {
                        targetsFound[(detection.id - 1) % 3] = true;

                        if (targetsFound[0] && targetsFound[1] && targetsFound[2]) {
                            visionPortal.stopStreaming();
                            return false;
                        }
                    }
                }
            }
            sleep(100);
        }

        visionPortal.stopStreaming();
        return true;
    }

    public boolean[] initWithController(boolean park)
    {
        long nextTimestamp = 0;
        boolean regular = true;
        boolean extra = true;
        boolean parkInside = park;

        while (!gamepad1.right_bumper)
        {
            if (gamepad1.x && gamepad1.timestamp > nextTimestamp) {
                nextTimestamp = gamepad1.timestamp + 300;
                regular = true;
                extra = false;
            }

            if (gamepad1.y && gamepad1.timestamp > nextTimestamp) {
                nextTimestamp = gamepad1.timestamp + 300;
                regular = false;
                extra = true;
            }

            if (gamepad1.a && gamepad1.timestamp > nextTimestamp) {
                nextTimestamp = gamepad1.timestamp + 300;
                parkInside = !parkInside;
            }

            telemetry.addLine("Press right bumper when you are done initializing.");
            telemetry.addData("CLOSE 2+0 | TRUSS 2+1 (x)", regular);
            telemetry.addData("TRUSS 2+3 (y)", extra);
            telemetry.addData("parkInside (a)", parkInside);
            telemetry.update();
            sleep(10);
        }

        telemetry.addLine("Initialization Complete");
        telemetry.update();

        return new boolean[]{regular, extra, parkInside};
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
        robot.CLAW.setPosition(0);
        robot.LFS.setPosition(0.97); // To swivel in more, increase this
        robot.RFS.setPosition(0.03); // To swivel in more, decrease this
        // Set CLAW to close position
        robot.CLAW.setPosition(0.5);
        // Lift upward
        lift(moveLiftByInches);

        // WAIT 500 MS
        // SWIVEL OUT
    }

    public void scorePixelsOnBackboard(boolean wrist)
    {
        if (wrist) {
            robot.WRIST.setPosition(0.73);
        }
        robot.CLAW.setPosition(0); // Set CLAW to closed position

        // WAIT 750 MS
        // LIFT 5 IN
    }

    public void resetForTeleOp(double dist)
    {
        robot.WRIST.setPosition(0.38); // Set WRIST to vertical position
        robot.CLAW.setPosition(0); // Set CLAW to open position
    }
    public void resetForTeleOp1(double dist)
    {
        robot.WRIST.setPosition(0.38); // Set WRIST to vertical position
        lift(8);
        robot.CLAW.setPosition(0); // Set CLAW to open position

    }
    public void resetForTeleOp2(double dist)
    {

        robot.LFS.setPosition(0.95); // To swivel in more, increase this
        robot.RFS.setPosition(0.05);// To swivel in more, decrease this
        sleep(50);
        lift(-(dist+8));
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
//        robot.IN.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        robot.IN.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        while (robot.IN.getCurrentPosition() > -33) {
//            robot.IN.setPower(-0.2);
//        }
//        robot.IN.setPower(0);


        // New function based on run to position
        robot.IN.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.IN.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.IN.setTargetPosition(-35 * 2);
        robot.IN.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.IN.setPower(-1);

    }

    public void intakeWhite1() {
        robot.IN.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        setDropdown(targetDropdownHeight);
        robot.IN.setPower(1);
    }
    public void intake1White1(int t) {
        robot.IN.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        setDropdown(t);
        robot.IN.setPower(.7);
    }

    // Extra step when wanting to intake a second white pixel
    public void intakeTwoWhite2() {
        robot.IN.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        targetDropdownHeight-=2;
        setDropdown(targetDropdownHeight);
    }
    public void intakeWhite2() {
        robot.IN.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        targetDropdownHeight--;
        setDropdown(targetDropdownHeight);
    }
    public void intakeWhite3() {
        robot.IN.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        // Set CLAW to close position
        setDropdown(0);
        robot.IN.setPower(-1);
    }

    public void intakeWhite4() {
        robot.IN.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        setDropdown(5);
        robot.IN.setPower(0);
    }

    public void pushDown() {

        robot.LL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.RL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        robot.LL.setPower(0.2 );
        robot.RL.setPower(0.2);
    }
    }