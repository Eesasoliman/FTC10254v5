package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.resources.DriveOpMode;
import org.firstinspires.ftc.teamcode.resources.RedColorPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous

public class REDback extends DriveOpMode {
    int imageNum;
    double backboardY = 53;

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);
        SampleMecanumDrive drive = initDriveOpMode();

        boolean[] driveVariables = initWithController(false);
        double parkY=0;
        if (driveVariables[1]){
            parkY=-12;
        }else{
            parkY=-60;
        }
        // Start Positions for Different Trajectories
        Pose2d startPose = new Pose2d(15.5, -63, Math.toRadians(270));
        Pose2d backboardPose = new Pose2d(50, -36, Math.toRadians(0));
        double liftHeight = 5.5;

        // Build all potential Trajectory Sequences
        double backVel=DriveConstants.MAX_VEL/4;

        TrajectorySequence path1 = drive.trajectorySequenceBuilder(startPose)
                //purple pixel
                .lineToConstantHeading(new Vector2d(17,-34))
                .lineToSplineHeading(new Pose2d(14.5, -30, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {intake();})
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {

                    prepareScoring(liftHeight);
                })
                .forward(5)

                // .waitSeconds(10000) use if working with robosupport

                //yellow pixel
//                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> {
                .lineToLinearHeading(new Pose2d(backboardY-5, -28.5, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(backboardY+10,-28.5,Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(.2, () -> {
                    scorePixelsOnBackboard(liftHeight);
                })
                .waitSeconds(1)
                .back(12,SampleMecanumDrive.getVelocityConstraint(backVel*4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .lineToConstantHeading(new Vector2d(52,parkY))
                .forward(5)
                .build();

        TrajectorySequence path2 = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(14, -34))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {intake();})
                .waitSeconds(0.5)
                .forward(5)


//                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {

                    prepareScoring(liftHeight);
                })
                .turn(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(backboardY-5, -35, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(backboardY+10,-35,Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    scorePixelsOnBackboard(liftHeight);
                })
                .waitSeconds(1)
                .back(12,SampleMecanumDrive.getVelocityConstraint(backVel*4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .lineToConstantHeading(new Vector2d(52,parkY))
                .forward(10)
                .build();

        TrajectorySequence path3 = drive.trajectorySequenceBuilder(startPose)
                //purple pixel
                .lineToSplineHeading(new Pose2d(30, -45, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(37, -28), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {intake();})
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    prepareScoring(liftHeight);
                })
                .forward(10)


                //yellow pixel
//                .UNSTABLE_addTemporalMarkerOffset(0.65, () -> {

                .lineToLinearHeading(new Pose2d(backboardY-5, -41.5, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(backboardY+10,-41.5,Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    scorePixelsOnBackboard(liftHeight);
                })
                .waitSeconds(1)
                .back(12,SampleMecanumDrive.getVelocityConstraint(backVel*4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .lineToConstantHeading(new Vector2d(52,parkY))
                .forward(10)
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
            if(driveVariables[0])
            {
                //yellow
                drive.followTrajectorySequence(path1);
                resetForTeleOp(liftHeight);
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
                resetForTeleOp(liftHeight);
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
                resetForTeleOp(liftHeight);
            }else{
                drive.followTrajectorySequence(path3Purple);
            }
        }
    }
}
