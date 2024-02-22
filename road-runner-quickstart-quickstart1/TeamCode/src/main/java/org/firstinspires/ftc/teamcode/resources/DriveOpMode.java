package org.firstinspires.ftc.teamcode.resources;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.HardwarePushBot;
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

    // New Robot's init code
    /**
    public boolean[] initWithController()
    {
        boolean blueSide = true;
        boolean backStage = true;
        boolean scoreYellowPixel = true;
        boolean scoreWhitePixels = true;
        boolean parkInside = true;

        while (!gamepad1.dpad_down || !gamepad2.dpad_down || !gamepad1.dpad_up || !gamepad2.dpad_up || !gamepad1.dpad_left || !gamepad2.dpad_left || !gamepad1.dpad_right || !gamepad2.dpad_right)
        {
            telemetry.addLine("Press \"dpad\" to end initWithController()");
            telemetry.addData("x -> blueSide", blueSide);
            telemetry.addData("y -> backStage", backStage);
            telemetry.addData("a -> scoreYellowPixel", scoreYellowPixel);
            telemetry.addData("b -> scoreWhitePixels", scoreWhitePixels);
            telemetry.addData("right_bumper/left_bumper -> parkInside", parkInside);
            telemetry.update();

            if (gamepad1.x || gamepad2.x)
            {
                blueSide = !blueSide;
                sleep(250);
            }

            if (gamepad1.y || gamepad2.y)
            {
                backStage = !backStage;
                sleep(250);
            }

            if (gamepad1.a || gamepad2.a)
            {
                backStage = !backStage;
                sleep(250);
            }

            if (gamepad1.b || gamepad2.b)
            {
                backStage = !backStage;
                sleep(250);
            }

            if (gamepad1.right_bumper || gamepad2.right_bumper)
            {
                parkInside = true;
            }

            if (gamepad1.left_bumper || gamepad1.left_bumper)
            {
                parkInside = false;
            }
        }

        return new boolean[]{blueSide, backStage, scoreYellowPixel, scoreWhitePixels, parkInside};
    }
     */

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

    public TrajectorySequence[] getTrajectories(int imageNum, int multiplier)
    {
        // MULTIPLIER -> 1 for Blue Side
        // MULTIPLIER -> -1 for Red Side

        final Pose2d FRONT = new Pose2d(FRONT_X, Y * multiplier, Math.toRadians(90 * multiplier));
        final Pose2d BACK = new Pose2d(BACK_X, Y * multiplier, Math.toRadians(90 * multiplier));

        Pose2d BOARD = new Pose2d(50, 36 * multiplier, Math.toRadians(0));
        Pose2d WHITE = new Pose2d(-55, 24 * multiplier, Math.toRadians(180));
        Pose2d FRONT_END = new Pose2d(0, 0, Math.toRadians(0));
        Pose2d BACK_END = new Pose2d(0, 0, Math.toRadians(0));
        if (imageNum == 1)
        {
            BOARD = new Pose2d(BOARD.getX(), BOARD.getY() + 10, Math.toRadians(0));
            WHITE = new Pose2d(WHITE.getX(), WHITE.getY() + 10, Math.toRadians(180));
            FRONT_END = new Pose2d(-41, 30 * multiplier, Math.toRadians(0));
            BACK_END = new Pose2d(7, 30 * multiplier, Math.toRadians(0));
        }
        else if (imageNum == 2 || imageNum == 3 || imageNum == 4)
        {
            BOARD = new Pose2d(BOARD.getX(), BOARD.getY(), Math.toRadians(0));
            WHITE = new Pose2d(WHITE.getX(), WHITE.getY(), Math.toRadians(180));
            FRONT_END = new Pose2d(-36, 38  * multiplier, Math.toRadians(270 * multiplier));
            BACK_END = new Pose2d(12, 38  * multiplier, Math.toRadians(270 * multiplier));
        }
        else if (imageNum == 5)
        {
            BOARD = new Pose2d(BOARD.getX(), BOARD.getY() - 10, Math.toRadians(0));
            WHITE = new Pose2d(WHITE.getX(), WHITE.getY() - 10, Math.toRadians(180));
            FRONT_END = new Pose2d(-31, 30 * multiplier, Math.toRadians(0));
            BACK_END = new Pose2d(31, 30  * multiplier, Math.toRadians(0));
        }

        TrajectorySequence Back1 = drive.trajectorySequenceBuilder(BACK)
                .strafeLeft(4 * multiplier)
                .setReversed(true)
                .splineToLinearHeading(BACK_END, Math.toRadians(180)) // Turn while moving to correct pos
            .build();
        TrajectorySequence Back2 = drive.trajectorySequenceBuilder(BACK)
            .lineToConstantHeading(new Vector2d(BACK_END.getX(), BACK_END.getY())) // Move to correct pos
            .build();
        TrajectorySequence Back3 = drive.trajectorySequenceBuilder(BACK)
                .strafeLeft(4 * multiplier)
                .setReversed(true)
                .splineToLinearHeading(BACK_END, Math.toRadians(0)) // Turn while moving to correct pos
            .build();

        TrajectorySequence Front1 = drive.trajectorySequenceBuilder(FRONT)
            .splineToLinearHeading(FRONT_END, Math.toRadians(0)) // Turn while moving to correct pos
            .build();
        TrajectorySequence Front2 = drive.trajectorySequenceBuilder(FRONT)
            .lineToConstantHeading(new Vector2d(FRONT_END.getX(), FRONT_END.getY()))
            .build();
        TrajectorySequence Front3 = drive.trajectorySequenceBuilder(FRONT)
                .strafeLeft(4 * multiplier)
                .setReversed(true)
                .splineToLinearHeading(FRONT_END, Math.toRadians(0)) // Turn while moving to correct pos
            .build();

        // Make sure that YellowBack and YellowFront end in BOARD
        TrajectorySequence YellowBack = drive.trajectorySequenceBuilder(BACK_END)
            .splineToSplineHeading(BOARD, Math.toRadians(0))
            .splineToSplineHeading(
                new Pose2d(BOARD.getX() + 10, BOARD.getY()),
                Math.toRadians(0),
                SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
            )
            .build();

        TrajectorySequence YellowFront = drive.trajectorySequenceBuilder(FRONT_END)
            .splineToSplineHeading(new Pose2d(-36, 12 * multiplier), Math.toRadians(270 * multiplier))
            .splineToSplineHeading(new Pose2d(12, 12 * multiplier), Math.toRadians(0))
            .splineToSplineHeading(BOARD, Math.toRadians(0))
                .splineToSplineHeading(
                        new Pose2d(BOARD.getX() + 10, BOARD.getY()),
                        Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
            .build();

        // Call this trajectory as many times as needed
        TrajectorySequence ScoreWhite = drive.trajectorySequenceBuilder(WHITE)
            .splineToSplineHeading(new Pose2d(-48, 12 * multiplier), Math.toRadians(270 * multiplier))
            .splineToSplineHeading(new Pose2d(48, 12 * multiplier), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(BOARD.getX(), BOARD.getY()), Math.toRadians(270 * multiplier))
                .splineToSplineHeading(
                        new Pose2d(BOARD.getX() + 10, BOARD.getY()),
                        Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
            .build();

        TrajectorySequence GetWhite = drive.trajectorySequenceBuilder(BOARD)
            .splineToSplineHeading(new Pose2d(48, 12 * multiplier), Math.toRadians(270 * multiplier))
            .splineToSplineHeading(new Pose2d(-48, 12 * multiplier), Math.toRadians(180))
            .splineToSplineHeading(new Pose2d(WHITE.getX(), WHITE.getY()), Math.toRadians(270 * multiplier))
            .build();

        return new TrajectorySequence[]{Back1, Back2, Back3, Front1, Front2, Front3, YellowFront, YellowBack, ScoreWhite, GetWhite};
    }

    public void prepareScoring(double moveLiftByInches)
    {
        robot.LFS.setPosition(0.95);
        robot.RFS.setPosition(0.05);
        // Lock carriage servos
        robot.BPS.setPosition(.4);
        robot.FPS.setPosition(0);
        // Lift upward
        lift(true, moveLiftByInches);
        sleep(800);
        // Swivel out
        robot.LFS.setPosition(0.45);//higher
        robot.RFS.setPosition(0.55);//lower
    }

    public void scorePixelsOnBackboard(double moveLiftByInches)
    {
        // Release carriage servos
        robot.BPS.setPosition(0.40);
        robot.FPS.setPosition(0.40);
        sleep(500);
        lift(true, 5.5);
//        sleep(1000);
        // Swivel in
//        robot.LFS.setPosition(0.95);
//        robot.RFS.setPosition(0.05);
//        sleep(500);
//        lift(false, moveLiftByInches-.5);
    }

    public void lift(boolean up, double inches)
    {
        // https://gm0.org/en/latest/docs/common-mechanisms/dead-wheels.html
        // https://ftc-tech-toolbox.vercel.app/docs/odo/md
        // GO Builda Encoder Resolution: 2000 Countable Ticks per Revolution

        final double ENCODER_RESOLUTION = 384.5;
        final double CountsPerIN = 83.34;
        double power = 0.7;
        if (up)
        {
            power *= -1;
        }

        int ticksToMove = (int) ((inches *CountsPerIN));

        robot.LL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.RL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        robot.LL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.RL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

       while (Math.abs(robot.LL.getCurrentPosition()) < ticksToMove && Math.abs(robot.RL.getCurrentPosition()) < ticksToMove) {
        // while (Math.abs(robot.RL.getCurrentPosition()) < ticksToMove) {
           robot.RL.setPower(power);
           robot.LL.setPower(power);
           telemetry.addData("LL", robot.LL.getCurrentPosition());
           telemetry.addData("RL", robot.RL.getCurrentPosition());
           telemetry.update();
        }
        robot.RL.setPower(0);
        robot.LL.setPower(0);
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
