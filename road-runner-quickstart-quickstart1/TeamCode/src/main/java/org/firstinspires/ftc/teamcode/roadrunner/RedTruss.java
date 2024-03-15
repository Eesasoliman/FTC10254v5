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
public class RedTruss extends DriveOpMode {
    int imageNum;
    double parkY;
    double whitepixel = -36;
    double backboardY = 50;
    double afterdrop = 1;
    double liftHeight = 7.8;
    double dropdownHeight = 0;
    double backboardX = 50;
    double backboardVel = DriveConstants.MAX_VEL / 4;
    double backboardAcc = DriveConstants.MAX_ACCEL / 2;
    double backVel=DriveConstants.MAX_VEL/4;

    Pose2d startPose = new Pose2d(-36, -65.25, Math.toRadians(270));

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

        TrajectorySequence purple1 = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(3)
                .lineToLinearHeading(new Pose2d(-48, -36, Math.toRadians(270)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    purpleIntake();
                })
                .waitSeconds(0.5)
                .forward(12)
//                .lineToConstantHeading(new Vector2d(-38, -8))
                .build();
        TrajectorySequence purple2 = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(4)
                .lineToConstantHeading(new Vector2d(-36, -34.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    purpleIntake();
                })
                .waitSeconds(1)
                .forward(8)
                .build();
        TrajectorySequence purple3 = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-37.5, -40, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-34, -29), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Start Intake
                    purpleIntake();
                })
                .waitSeconds(0.5)
                .forward(7)
                .build();
        TrajectorySequence yellow1 = drive.trajectorySequenceBuilder(purple1.end())
                .lineToLinearHeading(new Pose2d(-55, -36,Math.toRadians(0)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-37.5, -59))
//                .setConstraints(
//                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .lineToConstantHeading(new Vector2d(24, -59))
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    prepareScoring(liftHeight);
                })
                .lineToConstantHeading(new Vector2d(backboardY-5, -26))
                .splineToConstantHeading(new Vector2d(backboardY+7, -28), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorePixelsOnBackboard();
                })
                .build();
        TrajectorySequence yellow2 = drive.trajectorySequenceBuilder(purple2.end())
                .lineToLinearHeading(new Pose2d(-60, -37,Math.toRadians(0)))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-37.5, -58, Math.toRadians(0)), Math.toRadians(0))
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .lineToConstantHeading(new Vector2d(24, -58))
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    prepareScoring(liftHeight);
                })
                .splineToConstantHeading(new Vector2d(backboardY-5, -33), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(backboardY+10, -33),
                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorePixelsOnBackboard();
                })
                .waitSeconds(afterdrop)
                .build();
        TrajectorySequence yellow3 = drive.trajectorySequenceBuilder(purple3.end())
                .forward(7)
                .splineToLinearHeading(new Pose2d(-37.5, -58, Math.toRadians(0)), Math.toRadians(270))
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .lineToConstantHeading(new Vector2d(24, -58))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    // Prepare Lift
                    prepareScoring(liftHeight);
                })
                .splineToConstantHeading(new Vector2d(backboardY-5, -36.5), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(backboardY+5, -36.5),
                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Drop Pixels
                    scorePixelsOnBackboard();
                })
                .waitSeconds(afterdrop)
                .build();
        TrajectorySequence white1 = drive.trajectorySequenceBuilder(purple1.end())
                // Get
                .splineToConstantHeading(new Vector2d(-12, -48), Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(-48, -48))
                .splineToConstantHeading(new Vector2d(-60, -36), Math.toRadians(270))

                // Return
                .splineToConstantHeading(new Vector2d(-48, -52), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(24, -52))
                .splineToConstantHeading(new Vector2d(yellow1.end().getX(), yellow1.end().getY()), Math.toRadians(270))
                .build();
        TrajectorySequence white2 = drive.trajectorySequenceBuilder(purple2.end())
                // Get
                .splineToConstantHeading(new Vector2d(-12, -48), Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(-48, -48))
                .splineToConstantHeading(new Vector2d(-60, -36), Math.toRadians(270))

                // Return
                .splineToConstantHeading(new Vector2d(-48, -52), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(24, -52))
                .splineToConstantHeading(new Vector2d(yellow2.end().getX(), yellow2.end().getY()), Math.toRadians(270))
                .build();
        TrajectorySequence white3 = drive.trajectorySequenceBuilder(purple3.end())
                // Get
                .splineToConstantHeading(new Vector2d(-12, -48), Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(-48, -48))
                .splineToConstantHeading(new Vector2d(-60, -36), Math.toRadians(270))

                // Return
                .splineToConstantHeading(new Vector2d(-48, -52), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(24, -52))
                .splineToConstantHeading(new Vector2d(yellow3.end().getX(), yellow3.end().getY()), Math.toRadians(270))
                .build();
        TrajectorySequence park1 = drive.trajectorySequenceBuilder(white1.end())
                .back(12)
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> {
                    resetForTeleOp(liftHeight);
                })
                .lineToConstantHeading(new Vector2d(52,parkY))
                .forward(10)
                .build();
        TrajectorySequence park2 = drive.trajectorySequenceBuilder(white2.end())
                .back(12)
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> {
                    resetForTeleOp(liftHeight);
                })
                .lineToConstantHeading(new Vector2d(52,parkY))
                .forward(10)
                .build();
        TrajectorySequence park3 = drive.trajectorySequenceBuilder(white3.end())
                .back(12)
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> {
                    resetForTeleOp(liftHeight);
                })
                .lineToConstantHeading(new Vector2d(52,parkY))
                .forward(10)
                .build();

//        initAprilTagProcessor();
        RedColorPipeline pipeline = startRedCamera();
        while (!isStopRequested() && opModeInInit()) {
            sleep(1000);
            imageNum = pipeline.getImageNum();
            telemetry.addData("imageNum", imageNum);
            telemetry.update();
        }

        waitForStart();
        drive.setPoseEstimate(startPose);

        TrajectorySequence purple = null;
        TrajectorySequence yellow = null;
        TrajectorySequence white = null;
        TrajectorySequence park = null;
        imageNum = 3;

        if (imageNum == 1) {
            purple = purple1;
            yellow = yellow1;
            white = white1;
            park = park1;
        } else if (imageNum == 2 || imageNum == 3 || imageNum == 4) {
            purple = purple2;
            yellow = yellow2;
            white = white2;
            park = park2;
        } else if (imageNum == 5) {
            purple = purple3;
            yellow = yellow3;
            white = white3;
            park = park3;
        }

        drive.followTrajectorySequence(purple);
        if (driveVariables[0]) {
            drive.followTrajectorySequence(yellow);
//            drive.followTrajectorySequence(relocalize(yellow.end(), 0));
            if (driveVariables[2]) {
                // Follow White Twice
                drive.followTrajectorySequence(white);
 //               drive.followTrajectorySequence(relocalize(white.end(), 0));
                drive.followTrajectorySequence(white);
            } else if (driveVariables[1]) {
                // Follow White Once
                drive.followTrajectorySequence(white);
            }
        }
        drive.followTrajectorySequence(park);
    }
}