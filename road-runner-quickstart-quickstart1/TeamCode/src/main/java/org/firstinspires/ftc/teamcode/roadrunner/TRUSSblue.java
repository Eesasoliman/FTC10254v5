package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.resources.BlueColorPipeline;
import org.firstinspires.ftc.teamcode.resources.DriveOpMode;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous

public class TRUSSblue extends DriveOpMode {
    int imageNum;

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);
        SampleMecanumDrive drive = initDriveOpMode();

        boolean[] driveVariables = initWithController(true);
        double liftHeight = 7.8;
        double parkY=0;
        double backboardY = 50;
        if (driveVariables[1]){
            parkY=8;
        }else{
            parkY=60;
        }
        double backVel = DriveConstants.MAX_VEL/4;

        Pose2d startPose = new Pose2d(-37.5, 63, Math.toRadians(90));
        // Build all potential Trajectory Sequences
        TrajectorySequence path1 = drive.trajectorySequenceBuilder(startPose)
                // Purple Pixel
                .lineToSplineHeading(new Pose2d(-37.5, 40, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-35, 29), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intake();
                })
                .waitSeconds(0.5)

                // Yellow Pixel
                .forward(10)
                .lineToLinearHeading(new Pose2d(-37.5, 58.5, Math.toRadians(0)))
                .lineToConstantHeading(
                        new Vector2d(30, 58.5),
                        SampleMecanumDrive.getVelocityConstraint(backVel*2, DriveConstants.MAX_ANG_VEL/2, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .UNSTABLE_addTemporalMarkerOffset(-0.8, () -> {
                    prepareScoring(liftHeight);
                })

                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .lineToConstantHeading(new Vector2d(backboardY-5, 42))
                .lineToConstantHeading(
                        new Vector2d(backboardY+5, 42))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Drop Pixels
                    scorePixelsOnBackboard(liftHeight);
                })
                .waitSeconds(1)

                // Park
                .back(12)
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> {
                    resetForTeleOp(liftHeight);
                })
                .lineToConstantHeading(new Vector2d(52,parkY))
            .strafeLeft(5)
                .forward(6)
                .build();

        TrajectorySequence path2 = drive.trajectorySequenceBuilder(startPose)
                // Purple Pixel
                .lineToConstantHeading(new Vector2d(-37, 33.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intake();
                })
                .waitSeconds(0.5)

                // Yellow Pixel
//                .splineToConstantHeading(new Vector2d(-43, 27), 90) use if it hits truss
                .lineToLinearHeading(new Pose2d(-37, 58, Math.toRadians(0)))
                .setConstraints(
                    SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToConstantHeading(new Vector2d(24, 58))
                .UNSTABLE_addTemporalMarkerOffset(.2, () -> {
                    // Prepare Lift
                    prepareScoring(liftHeight);
                })
                .lineToConstantHeading(new Vector2d(backboardY-5, 32))
                .lineToConstantHeading(new Vector2d(backboardY+4, 32))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Drop Pixels
                    scorePixelsOnBackboard(liftHeight);
                })
                .waitSeconds(1)

                .setConstraints(
                    SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                // Park
                .back(12)
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> {
                    resetForTeleOp(liftHeight);
                })
                .lineToConstantHeading(new Vector2d(52, parkY))
                .forward(7)
                .build();

        TrajectorySequence path3 = drive.trajectorySequenceBuilder(startPose)
                // Purple Pixel
                .strafeLeft(3)
                .splineToConstantHeading(new Vector2d(-48, 36), Math.toRadians(270))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    intake();
                })

                // Yellow Pixel
                .lineToLinearHeading(new Pose2d(-37.5, 58))
                .lineToConstantHeading(new Vector2d(24, 58))
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    // Prepare Lift
                    prepareScoring(liftHeight);
                })
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToConstantHeading(new Vector2d(backboardY-5, 26.5))
                .lineToConstantHeading(new Vector2d(backboardY+5, 26.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Drop Pixels
                    scorePixelsOnBackboard(liftHeight);
                })
                .waitSeconds(1)

                // Park
                .setConstraints(
                    SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .back(14)
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> {
                    resetForTeleOp(liftHeight);
                })
                .lineToConstantHeading(new Vector2d(52,parkY))
                .forward(8)
                .build();

        TrajectorySequence path1Purple = drive.trajectorySequenceBuilder(startPose)
                // Purple Pixel
                .lineToLinearHeading(new Pose2d(-39, 36, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-36, 29, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Start Intake
                    intake();
                })
                .waitSeconds(1)
                .forward(5)
                .build();

        TrajectorySequence path2Purple = drive.trajectorySequenceBuilder(startPose)
                //purple pixel
                .lineToConstantHeading(new Vector2d(-36, 34))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {intake();})
                .waitSeconds(1)
                .forward(5)
                .build();

        TrajectorySequence path3Purple = drive.trajectorySequenceBuilder(startPose)
                //Purple pixel
                .lineToSplineHeading(new Pose2d(-37.5, 40, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(-38, 29), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {intake();})
                .waitSeconds(1)
                .forward(5)
                .build();

        BlueColorPipeline pipeline = startBlueCamera();
        while (opModeInInit())
        {
            sleep(1000);
            imageNum = pipeline.getImageNum();
            telemetry.addData("imageNum", imageNum);
            telemetry.update();
        }

        waitForStart();
        drive.setPoseEstimate(startPose);

        if (imageNum == 1)
        {
            if(driveVariables[0])
            {
                //yellow
                drive.followTrajectorySequence(path1);
            }else{
                drive.followTrajectorySequence(path1Purple);
            }

        }
        else if (imageNum == 2 || imageNum == 3 || imageNum == 4)
        {

            if(driveVariables[0])
            {
                //yellow
                drive.followTrajectorySequence(path2);
            }else{
                drive.followTrajectorySequence(path2Purple);
            }
        }
        else if (imageNum == 5)
        {
            if(driveVariables[0])
            {
                //yellow
                drive.followTrajectorySequence(path3);
            }else{
                drive.followTrajectorySequence(path3Purple);
            }
        }
    }

}
