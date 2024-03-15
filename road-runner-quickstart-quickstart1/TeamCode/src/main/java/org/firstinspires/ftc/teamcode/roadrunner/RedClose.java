package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.resources.RedColorPipeline;
import org.firstinspires.ftc.teamcode.resources.DriveOpMode;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class RedClose extends DriveOpMode {
    int imageNum;
    double parkY;
    double liftHeight = 7.8;
    double backboardX = 50;
    double slowVel = DriveConstants.MAX_VEL / 1.35;
    double slowAcc = DriveConstants.MAX_ACCEL;
    Pose2d startPose = new Pose2d(12, -63, Math.toRadians(270));

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);
        SampleMecanumDrive drive = initDriveOpMode();

        boolean[] driveVariables = initWithController(true);
        if (driveVariables[3]) {
            parkY = -8;
        } else {
            parkY = -60;
        }

        Pose2d backboard1 = new Pose2d(backboardX, -28.5, Math.toRadians(0));
        Pose2d backboard2 = new Pose2d(backboardX, -35, Math.toRadians(0));
        Pose2d backboard3 = new Pose2d(backboardX, -41.5, Math.toRadians(0));

        telemetry.addLine("Building Trajectories");
        telemetry.update();
        // Build all potential Trajectory Sequences
        TrajectorySequence purple1 = drive.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(17,-34), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(14.5, -30, Math.toRadians(0)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { purpleIntake(); })
                .build();
        TrajectorySequence purple2 = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(14, -34))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { purpleIntake(); })
                .forward(5)
                .build();
        TrajectorySequence purple3 = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(30, -45, Math.toRadians(0)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(37, -28), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { purpleIntake(); })
                .build();
        TrajectorySequence yellow1 = drive.trajectorySequenceBuilder(purple1.end())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { prepareScoring(liftHeight); })
                .lineToLinearHeading(new Pose2d(backboardX, -28.5, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { scorePixelsOnBackboard(liftHeight); })
                .waitSeconds(1)
                .build();
        TrajectorySequence yellow2 = drive.trajectorySequenceBuilder(purple2.end())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { prepareScoring(liftHeight); })
                .lineToLinearHeading(new Pose2d(backboardX, -35, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { scorePixelsOnBackboard(liftHeight); })
                .waitSeconds(1)
                .build();
        TrajectorySequence yellow3 = drive.trajectorySequenceBuilder(purple3.end())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { prepareScoring(liftHeight); })
                .lineToLinearHeading(new Pose2d(backboardX, -41.5, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { scorePixelsOnBackboard(liftHeight); })
                .waitSeconds(1)
                .build();
        TrajectorySequence white1 = drive.trajectorySequenceBuilder(backboard1)
                // Get
                .strafeLeft(0.1)
                .splineToConstantHeading(new Vector2d(24, -7), Math.toRadians(180))
                .setConstraints(
                    SampleMecanumDrive.getVelocityConstraint(slowVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(slowAcc))
                .lineToConstantHeading(new Vector2d(-24,-7))
                .splineToConstantHeading(new Vector2d(-53, -14), Math.toRadians(270))
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(1)

                .UNSTABLE_addTemporalMarkerOffset(0,() -> { intakeTwoWhite(); })

                // Return
                .strafeLeft(0.1)
                .splineToConstantHeading(new Vector2d(-24, -7), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(24, -7))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> { resumeStreaming(); })
                .splineToSplineHeading(backboard1, Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(slowVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(slowAcc))
                .waitSeconds(1)
                .build();
        TrajectorySequence white2 = drive.trajectorySequenceBuilder(backboard2)
                // Get
                .strafeLeft(0.1)
                .splineToConstantHeading(new Vector2d(24, -7), Math.toRadians(180))
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(slowVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(slowAcc))
                .lineToConstantHeading(new Vector2d(-48, -7))
                .splineToConstantHeading(new Vector2d(-60, -36), Math.toRadians(270))
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(1)

                .UNSTABLE_addTemporalMarkerOffset(0,() -> { intakeTwoWhite(); })

                // Return
                .strafeLeft(0.1)
                .splineToConstantHeading(new Vector2d(-48, -7), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(24, -7))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> { resumeStreaming(); })
                .splineToSplineHeading(backboard2, Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(slowVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(slowAcc))
                .build();
        TrajectorySequence white3 = drive.trajectorySequenceBuilder(backboard3)
                // Get
                .strafeLeft(0.1)
                .splineToConstantHeading(new Vector2d(24, -7), Math.toRadians(180))
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(slowVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(slowAcc))
                .lineToConstantHeading(new Vector2d(-48, -7))
                .splineToConstantHeading(new Vector2d(-60, -36), Math.toRadians(270))
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(1)

                .UNSTABLE_addTemporalMarkerOffset(0,() -> { intakeTwoWhite(); })

                // Return
                .strafeLeft(0.1)
                .splineToConstantHeading(new Vector2d(-48, -7), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(24, -7))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> { resumeStreaming(); })
                .splineToSplineHeading(backboard3, Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(slowVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(slowAcc))
                .build();
        TrajectorySequence park1 = drive.trajectorySequenceBuilder(white1.end())
                .back(0.1)
                .splineToConstantHeading(new Vector2d(52, parkY), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> { resetForTeleOp(liftHeight); })
                .forward(10)
                .build();
        TrajectorySequence park2 = drive.trajectorySequenceBuilder(white2.end())
                .back(0.1)
                .splineToConstantHeading(new Vector2d(52, parkY), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> { resetForTeleOp(liftHeight); })
                .forward(10)
                .build();
        TrajectorySequence park3 = drive.trajectorySequenceBuilder(white3.end())
                .back(0.1)
                .splineToConstantHeading(new Vector2d(52, parkY), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> { resetForTeleOp(liftHeight); })
                .forward(10)
                .build();

        telemetry.addLine("Starting Camera");
        telemetry.update();
        initAprilTagProcessor();

        RedColorPipeline pipeline = startRedCamera();
        while (!isStopRequested() && opModeInInit()) {
            sleep(500);
            imageNum = pipeline.getImageNum();
            telemetry.addData("imageNum", imageNum);
            telemetry.update();
        }

        setDropdown(5);
        waitForStart();
        drive.setPoseEstimate(startPose);

        TrajectorySequence purple = null;
        TrajectorySequence yellow = null;
        TrajectorySequence white = null;
        TrajectorySequence park = null;
        Pose2d backboardPose = null;

        if (imageNum == 1) {
            purple = purple1;
            yellow = yellow1;
            white = white1;
            park = park1;
            backboardPose = backboard1;
        } else if (imageNum == 2 || imageNum == 3 || imageNum == 4) {
            purple = purple2;
            yellow = yellow2;
            white = white2;
            park = park2;
            backboardPose = backboard2;
        } else if (imageNum == 5) {
            purple = purple3;
            yellow = yellow3;
            white = white3;
            park = park3;
            backboardPose = backboard3;
        }

        drive.followTrajectorySequence(purple);
        if (driveVariables[0]) {
            drive.followTrajectorySequence(yellow);
            drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(relocalize(false))
                            .splineToSplineHeading(backboardPose, Math.toRadians(0))
                            .build());
            if (driveVariables[2]) {
                // Follow White Twice
                setTargetDropdownHeight(4);
                drive.followTrajectorySequence(white);
                drive.followTrajectorySequence(
                        drive.trajectorySequenceBuilder(relocalize(false))
                                .splineToSplineHeading(backboardPose, Math.toRadians(0))
                                .build());
                setTargetDropdownHeight(3);
                drive.followTrajectorySequence(white);
            } else if (driveVariables[1]) {
                // Follow White Once
                setTargetDropdownHeight(4);
                drive.followTrajectorySequence(white);
            }
        }
        drive.followTrajectorySequence(park);
    }
}
