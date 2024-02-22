package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.google.ar.core.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.resources.DriveOpMode;
import org.firstinspires.ftc.teamcode.resources.RedColorPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous

public class NEWBACKRED extends DriveOpMode {
    int imageNum;
    double backboardY = 53;

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);
        drive = initDriveOpMode();

        boolean[] driveVariables = initWithController(false);
        double parkY=0;
        if (driveVariables[1]){
            parkY=-12;
        }else{
            parkY=-58;
        }
        // Start Positions for Different Trajectories
        Pose2d startPose = new Pose2d(15.5, -63, Math.toRadians(270));
        Pose2d backboardPose = new Pose2d(50, -36, Math.toRadians(0));
        double liftHeight = 5.5;

        // Build all potential Trajectory Sequences
        double backVel=DriveConstants.MAX_VEL/4;

        TrajectorySequence purple1 = drive.trajectorySequenceBuilder(startPose)
                //purple pixel
                .lineToConstantHeading(new Vector2d(17,-34))
                .lineToSplineHeading(new Pose2d(14.5, -30, Math.toRadians(0)))
                .build();

        TrajectorySequence yellow1 = drive.trajectorySequenceBuilder(new Pose2d(14.5, -30, Math.toRadians(0)))
                .forward(5)
                .lineToLinearHeading(new Pose2d(backboardY-5, -28.5, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(backboardY+10,-28.5,Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence purple2 = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(14, -34))
                .build();

        TrajectorySequence yellow2 = drive.trajectorySequenceBuilder(new Pose2d(14, -34, Math.toRadians(270)))
                .forward(6)
                .turn(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(backboardY-5, -35, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(backboardY+10,-35,Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence purple3 = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(30, -45, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(37, -28), Math.toRadians(90))
                .build();

        TrajectorySequence yellow3 = drive.trajectorySequenceBuilder(new Pose2d(37, -28, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(backboardY-5, -41.5, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(backboardY+10,-41.5,Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        TrajectorySequence path1Purple = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(2.07, () -> {intake();})
                .lineToConstantHeading(new Vector2d(17,-34))
                .lineToSplineHeading(new Pose2d(14.5, -30, Math.toRadians(0)))
                .waitSeconds(1)
                .forward(5)
                .build();

        TrajectorySequence path2Purple = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> {intake();})
                .lineToConstantHeading(new Vector2d(14, -34))
                .waitSeconds(0.5)
                .forward(5)

                .build();

        TrajectorySequence path3Purple = drive.trajectorySequenceBuilder(startPose)
                //purple pixel
                .UNSTABLE_addTemporalMarkerOffset(1.8, () -> {intake();})
                .lineToSplineHeading(new Pose2d(30, -45, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(37, -28), Math.toRadians(90))
                .waitSeconds(0.5)
                .forward(10)
                .build();

        RedColorPipeline pipeline = startRedCamera();
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

            //yellow
            if(driveVariables[0])
            {
                    //yellow
                    drive.followTrajectorySequence(purple1);
                    intake();
                    prepareScoring(liftHeight);
                    drive.followTrajectorySequence(yellow1);
                    sleep(100);
                    scorePixelsOnBackboard(liftHeight);
                    drive.followTrajectorySequence(
                            drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .back(12,SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                    .lineToConstantHeading(new Vector2d(52,parkY))
                                    .forward(5)
                            .build());

            }else{
                drive.followTrajectorySequence(path1Purple);
            }

        }
        else if (imageNum == 2 || imageNum == 3 || imageNum == 4)
        {

            if(driveVariables[0])
            {
                if (!driveVariables[2]) {
                    //yellow
                    drive.followTrajectorySequence(purple2);
                    intake();
                    prepareScoring(liftHeight);
                    drive.followTrajectorySequence(yellow2);
                    sleep(100);
                    scorePixelsOnBackboard(liftHeight);
                    drive.followTrajectorySequence(
                            drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .back(12,SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                    .lineToConstantHeading(new Vector2d(52,parkY))
                                    .forward(5)
                                    .build());
                }

            }else{
                drive.followTrajectorySequence(path2Purple);
            }
        }


        else if (imageNum == 5)
        {

            if(driveVariables[0])
            {
                if (!driveVariables[2]) {
                    //yellow
                    drive.followTrajectorySequence(purple3);
                    intake();
                    prepareScoring(liftHeight);
                    drive.followTrajectorySequence(yellow3);
                    sleep(100);
                    scorePixelsOnBackboard(liftHeight);
                    drive.followTrajectorySequence(
                            drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .back(12,SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                    .lineToConstantHeading(new Vector2d(52,parkY))
                                    .forward(5)
                                    .build());
                }


            }else{
                drive.followTrajectorySequence(path3Purple);
            }
        }

    }
}
