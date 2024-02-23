package org.firstinspires.ftc.teamcode.resources;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.HardwarePushBot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

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

    public static final int FRONT_X = -36;
    public static final int BACK_X = 12;
    public static final double Y = 63.7845;

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

    public boolean[] initWithController(boolean park)
    {
        boolean dropYellowPixel = true;
        boolean parkInside = park;

        while (!gamepad1.right_bumper)
        {
            if (gamepad1.x) {
                dropYellowPixel = true;
            }

            if (gamepad1.y) {
                dropYellowPixel = false;
            }

            if (gamepad1.a) {
                parkInside = true;
            }

            if (gamepad1.b) {
                parkInside = false;
            }

            // Press right bumper when you are done initializing.
            telemetry.addLine("dropYellowPixel: x - > true, y -> false");
            telemetry.addLine("parkInside: a - > true, b -> false");
            telemetry.addData("dropYellowPixel", dropYellowPixel);
            telemetry.addData("parkInside", parkInside);
            telemetry.update();

            sleep(50);
        }

        return new boolean[]{dropYellowPixel, parkInside, false, false};
    }

    public void prepareScoring(double moveLiftByInches)
    {
        // Swivel in
        robot.LFS.setPosition(0.95);
        robot.RFS.setPosition(0.05);
        // Lock carriage servos
        robot.BPS.setPosition(.4);
        robot.FPS.setPosition(0);
        // Lift upward
        lift(moveLiftByInches);
        sleep(500);
        // Swivel out
        robot.LFS.setPosition(0.51); //lower to make it go more out
        robot.RFS.setPosition(0.49); //higher to make it go more out
    }

    public void scorePixelsOnBackboard(double liftEndPositionInches)
    {
        robot.BPS.setPosition(0.40);
        robot.FPS.setPosition(0.40);
        sleep(250);
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
