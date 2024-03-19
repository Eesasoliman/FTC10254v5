package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.resources.ColorPipeline;
import org.firstinspires.ftc.teamcode.resources.DriveOpMode;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class BlueClose extends DriveOpMode {
    int imageNum;
    double parkY;
    double liftHeight = 5.5;
    double backboardX = 51.5;

    double stackY = 13;
    double slowVel = DriveConstants.MAX_VEL / 1.35;
    double slowAcc = DriveConstants.MAX_ACCEL;
    Pose2d startPose = new Pose2d(12, 63, Math.toRadians(90));

    @Override
    public void runOpMode() {

        telemetry.setMsTransmissionInterval(50);
        SampleMecanumDrive drive = initDriveOpMode();

        boolean[] driveVariables = initWithController(true);
        if (driveVariables[3]) {
            parkY = 8;
        } else {
            parkY = 60;
        }
        robot.CLAW.setPosition(0.5);
        robot.LFS.setPosition(0.95); // To swivel in more, increase this
        robot.RFS.setPosition(0.05);// To swivel in more, decrease this
        Pose2d backboard1 = new Pose2d(backboardX, 28.5, Math.toRadians(0));
        Pose2d backboard2 = new Pose2d(backboardX, 35, Math.toRadians(0));
        Pose2d backboard3 = new Pose2d(backboardX, 41.5, Math.toRadians(0));

        telemetry.addLine("Building Trajectories");
        telemetry.update();
        // Build all potential Trajectory Sequences
        TrajectorySequence purple1 = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(9.5,32), Math.toRadians(180))
//                .splineTocoHeading(new Pose2d(12.5, 30, Math.toRadians(0)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0, this::purpleIntake)
                .build();
        TrajectorySequence purple2 = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(18, 22.5, Math.toRadians(0)), Math.toRadians(-120))
                .UNSTABLE_addTemporalMarkerOffset(-.1, this::purpleIntake)
                .build();
        TrajectorySequence purple3 = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-80))
                .splineToSplineHeading(new Pose2d(30, 45, Math.toRadians(0)), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(34, 28), Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(0, this::purpleIntake)
                .build();

        TrajectorySequence white1 = drive.trajectorySequenceBuilder(backboard1)
                // Get
                .strafeRight(0.1)
                .splineToConstantHeading(new Vector2d(24, 2), Math.toRadians(180))
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(slowVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(slowAcc))
                .lineToConstantHeading(new Vector2d(-48, 2))
                .splineToConstantHeading(new Vector2d(-55, stackY +5), Math.toRadians(-270))
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .UNSTABLE_addTemporalMarkerOffset(0, this::intakeWhite1)
                .back(.7)
                .UNSTABLE_addTemporalMarkerOffset(0.3, this::intakeTwoWhite2)
                .forward(1.3)
                .waitSeconds(1)

                // Return
                .UNSTABLE_addTemporalMarkerOffset(0, this::intakeWhite3)
                .splineToConstantHeading(new Vector2d(-48, 7), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->{robot.CLAW.setPosition(0.5);})
                .UNSTABLE_addTemporalMarkerOffset(0.5, this::intakeWhite4)
                .lineToConstantHeading(new Vector2d(24, 7))
                .splineToSplineHeading(new Pose2d(backboardX-20, 41.5, Math.toRadians(0)), Math.toRadians(-270),
                        SampleMecanumDrive.getVelocityConstraint(slowVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(slowAcc))
                .build();
        TrajectorySequence white2 = drive.trajectorySequenceBuilder(backboard2)
                // Get
                .strafeRight(0.1)
                .splineToConstantHeading(new Vector2d(24, 7), Math.toRadians(180))
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(slowVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(slowAcc))
                .lineToConstantHeading(new Vector2d(-48, 7))
                .splineToConstantHeading(new Vector2d(-60, stackY), Math.toRadians(-270))
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .UNSTABLE_addTemporalMarkerOffset(0, this::intakeWhite1)
//                .back(.7)
                .UNSTABLE_addTemporalMarkerOffset(0.3, this::intakeTwoWhite2)
                .forward(.5)
                .UNSTABLE_addTemporalMarkerOffset(0, this::intakeWhite3)

                .strafeLeft(1)
                .strafeRight(2)
//                .waitSeconds(1)

                // Return
                .splineToConstantHeading(new Vector2d(-48, 7), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->{robot.CLAW.setPosition(0.5);})
                .UNSTABLE_addTemporalMarkerOffset(0.5, this::intakeWhite4)
                .lineToConstantHeading(new Vector2d(24, 7))
                .splineToSplineHeading(new Pose2d(backboardX-0, 28.5, Math.toRadians(0)), Math.toRadians(-270),
                        SampleMecanumDrive.getVelocityConstraint(slowVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(slowAcc))
                .build();
        TrajectorySequence white3 = drive.trajectorySequenceBuilder(backboard3)
                // Get
                .strafeRight(0.1)
                .splineToConstantHeading(new Vector2d(24, 2), Math.toRadians(180))
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(slowVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(slowAcc))
                .lineToConstantHeading(new Vector2d(-48, 2))
                .splineToConstantHeading(new Vector2d(-55, stackY +5), Math.toRadians(-270))
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .UNSTABLE_addTemporalMarkerOffset(0, this::intakeWhite1)
                .back(.7)
                .UNSTABLE_addTemporalMarkerOffset(0.3, this::intakeTwoWhite2)
                .forward(1.3)
                .waitSeconds(1)

                // Return
                .UNSTABLE_addTemporalMarkerOffset(0, this::intakeWhite3)
                .splineToConstantHeading(new Vector2d(-48, 7), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->{robot.CLAW.setPosition(0.5);})
                .UNSTABLE_addTemporalMarkerOffset(0.5, this::intakeWhite4)
                .lineToConstantHeading(new Vector2d(24, 7))
                .splineToSplineHeading(new Pose2d(backboardX-20, 41.5, Math.toRadians(0)), Math.toRadians(-270),
                        SampleMecanumDrive.getVelocityConstraint(slowVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(slowAcc))
                .build();
        TrajectorySequence park1 = drive.trajectorySequenceBuilder(white1.end())
                .back(0.1)
                .splineToConstantHeading(new Vector2d(52, parkY), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> resetForTeleOp(liftHeight))
//                .forward(10)
                .build();
        TrajectorySequence park2 = drive.trajectorySequenceBuilder(white2.end())
                .back(0.1)
                .splineToConstantHeading(new Vector2d(52, parkY), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> resetForTeleOp(liftHeight))
//                .forward(10)
                .build();
        TrajectorySequence park3 = drive.trajectorySequenceBuilder(white3.end())
                .back(0.1)
                .splineToConstantHeading(new Vector2d(52, parkY), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> resetForTeleOp(liftHeight))
//                .forward(10)
                .build();

        telemetry.addLine("Starting Cameras...");
        telemetry.update();

        ColorPipeline pipeline = startCameras(true);
        while (!isStopRequested() && opModeInInit()) {
            sleep(1000);
            imageNum = pipeline.getImageNum();
            telemetry.addData("imageNum", imageNum);
            telemetry.update();
        }

        setDropdown(5);
        waitForStart();
        closeColorPipelineCamera();

        drive.setPoseEstimate(startPose);

        TrajectorySequence purple = null;
        TrajectorySequence yellow = null;
        TrajectorySequence white = null;
        TrajectorySequence park = null;
        Pose2d backboardPose = null;
        Pose2d wbackboardPose = null;

        if (imageNum == 1) {
            purple = purple3;
            white = white3;
            park = park3;
            backboardPose = backboard3;
            wbackboardPose = backboard1;
        } else if (imageNum == 2 || imageNum == 3 || imageNum == 4) {
            purple = purple2;
            white = white2;
            park = park2;
            backboardPose = backboard2;
            wbackboardPose = backboard1;
        } else if (imageNum == 5) {
            purple = purple1;
            white = white1;
            park = park1;
            backboardPose = backboard1;
            wbackboardPose = backboard3;
        }

//        waitForAprilTagCamera();
        resumeStreaming();

        telemetry.addLine("Trajectory Started.");
        telemetry.update();
        drive.followTrajectorySequence(purple);
        if (driveVariables[0]) {
            Pose2d correctPose = relocalize(true);
            drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(correctPose)
                            .UNSTABLE_addTemporalMarkerOffset(0, () -> prepareScoring(liftHeight))
                            .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                                // Swivel out
                                robot.LFS.setPosition(0.60); // To swivel out more, decrease this
                                robot.RFS.setPosition(0.40); // To swivel out more, increase this
                            })
                            .lineToLinearHeading(backboardPose)
                            .UNSTABLE_addTemporalMarkerOffset(0, () -> scorePixelsOnBackboard(false))
                            .waitSeconds(.5)
                            .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                                resetForTeleOp(liftHeight);
                                         })
                            .UNSTABLE_addTemporalMarkerOffset(1, () -> lift(-liftHeight))
                            .build());

            if (driveVariables[1] || driveVariables[2]) {
                // Follow White Once
                setTargetDropdownHeight(4);
                drive.followTrajectorySequence(white);
                correctPose = relocalize(true);
                drive.followTrajectorySequence(
                        drive.trajectorySequenceBuilder(correctPose)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> prepareScoring(liftHeight))
                                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                                    // Swivel out
                                    robot.LFS.setPosition(0.60); // To swivel out more, decrease this
                                    robot.RFS.setPosition(0.40); // To swivel out more, increase this
                                })
                                .lineToLinearHeading(wbackboardPose)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> scorePixelsOnBackboard(true))
                                .waitSeconds(.5)
                                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                                    resetForTeleOp(liftHeight);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(1, () -> lift(-liftHeight))
                                .build());
            }
        }
        drive.followTrajectorySequence(park);
    }
}
