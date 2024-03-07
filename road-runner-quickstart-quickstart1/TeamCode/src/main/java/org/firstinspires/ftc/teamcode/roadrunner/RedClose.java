package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.resources.BlueColorPipeline;
import org.firstinspires.ftc.teamcode.resources.DriveOpMode;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Close", group = "RED")
public class RedClose extends DriveOpMode {
    int imageNum;
    double parkY;
    double liftHeight = 7.8;
    double dropdownHeight = 0;
    double backboardX = 50;
    double backboardVel = DriveConstants.MAX_VEL / 4;
    double backboardAcc = DriveConstants.MAX_ACCEL / 2;
    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

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

        // Build all potential Trajectory Sequences
        TrajectorySequence purple1 = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(17,-34))
                .lineToSplineHeading(new Pose2d(14.5, -30, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { purpleIntake(); })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { prepareScoring(liftHeight); })
                .forward(5)
                .build();
        TrajectorySequence purple2 = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(14, -34))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { purpleIntake(); })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { prepareScoring(liftHeight); })
                .forward(5)
                .build();
        TrajectorySequence purple3 = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(30, -45, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(37, -28), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { purpleIntake(); })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { prepareScoring(liftHeight); })
                .forward(10)
                .build();
        TrajectorySequence yellow1 = drive.trajectorySequenceBuilder(purple1.end())
                .lineToLinearHeading(new Pose2d(backboardX, -28.5, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { scorePixelsOnBackboard(liftHeight); })
                .waitSeconds(1)
                .build();
        TrajectorySequence yellow2 = drive.trajectorySequenceBuilder(purple2.end())
                .lineToLinearHeading(new Pose2d(backboardX, -35, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { scorePixelsOnBackboard(liftHeight); })
                .waitSeconds(1)
                .build();
        TrajectorySequence yellow3 = drive.trajectorySequenceBuilder(purple3.end())
                .lineToLinearHeading(new Pose2d(backboardX, -41.5, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { scorePixelsOnBackboard(liftHeight); })
                .waitSeconds(1)
                .build();
        TrajectorySequence white1 = drive.trajectorySequenceBuilder(yellow1.end())
                // Get
                .splineToConstantHeading(new Vector2d(24, -12), Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(-48, -12))
                .splineToConstantHeading(new Vector2d(-60, -36), Math.toRadians(270))

                // Return
                .splineToConstantHeading(new Vector2d(-48, -12), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(24, -12))
                .splineToConstantHeading(new Vector2d(yellow1.end().getX(), yellow1.end().getY()), Math.toRadians(270))
                .build();
        TrajectorySequence white2 = drive.trajectorySequenceBuilder(yellow2.end())
                // Get
                .splineToConstantHeading(new Vector2d(24, -12), Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(-48, -12))
                .splineToConstantHeading(new Vector2d(-60, -36), Math.toRadians(270))

                // Return
                .splineToConstantHeading(new Vector2d(-48, -12), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(24, -12))
                .splineToConstantHeading(new Vector2d(yellow2.end().getX(), yellow2.end().getY()), Math.toRadians(270))
                .build();
        TrajectorySequence white3 = drive.trajectorySequenceBuilder(yellow3.end())
                // Get
                .splineToConstantHeading(new Vector2d(24, -12), Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(-48, -12))
                .splineToConstantHeading(new Vector2d(-60, -36), Math.toRadians(270))

                // Return
                .splineToConstantHeading(new Vector2d(-48, -12), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(24, -12))
                .splineToConstantHeading(new Vector2d(yellow3.end().getX(), yellow3.end().getY()), Math.toRadians(270))
                .build();
        TrajectorySequence park1 = drive.trajectorySequenceBuilder(white1.end())
                .splineToConstantHeading(new Vector2d(52, parkY), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> { resetForTeleOp(liftHeight); })
                .forward(10)
                .build();
        TrajectorySequence park2 = drive.trajectorySequenceBuilder(white2.end())
                .splineToConstantHeading(new Vector2d(52, parkY), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> { resetForTeleOp(liftHeight); })
                .forward(10)
                .build();
        TrajectorySequence park3 = drive.trajectorySequenceBuilder(white3.end())
                .splineToConstantHeading(new Vector2d(52, parkY), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> { resetForTeleOp(liftHeight); })
                .forward(10)
                .build();

        initAprilTagProcessor();
        BlueColorPipeline pipeline = startBlueCamera();
        while (opModeInInit()) {
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
            drive.followTrajectorySequence(relocalize(yellow.end(), 0));
            if (driveVariables[2]) {
                // Follow White Twice
                drive.followTrajectorySequence(white);
                drive.followTrajectorySequence(relocalize(white.end(), 0));
                drive.followTrajectorySequence(white);
            } else if (driveVariables[1]) {
                // Follow White Once
                drive.followTrajectorySequence(white);
            }
        }
        drive.followTrajectorySequence(park);
    }
}
