package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.google.ar.core.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.resources.DriveOpMode;
import org.firstinspires.ftc.teamcode.resources.RedColorPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous

public class NEWFRONTRED extends DriveOpMode {
    int imageNum;

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);
        drive = initDriveOpMode();

        boolean[] driveVariables = initWithController(true);
        double liftHeight = 5;
        double backboardY = 50;
        double afterdrop = .7;
        double parkY = 0;
        if (driveVariables[1]){
            parkY = -10;
        }else{
            parkY = -60;
        }
        double backVel=DriveConstants.MAX_VEL/4;
        // Start Positions for Different Trajectories
        Pose2d startPose = new Pose2d(-37.5, -63, Math.toRadians(270));
        // Build all potential Trajectory Sequences
        TrajectorySequence purple1 = drive.trajectorySequenceBuilder(startPose)
                // Purple Pixel
                .strafeRight(3)
                .lineToLinearHeading(new Pose2d(-38, -30, Math.toRadians(0)))
                .build();
        TrajectorySequence yellow1 = drive.trajectorySequenceBuilder(new Pose2d(-38, -30, Math.toRadians(0)))
                .back(2)
                .lineToConstantHeading(new Vector2d(-38, -58))//change
                // Yellow Pixel
                .lineToConstantHeading(new Vector2d(55, -58))

//                .lineToConstantHeading(new Vector2d(35, -60))
//                .lineToConstantHeading(new Vector2d(backboardY-5, -26))
//                .splineToConstantHeading(new Vector2d(backboardY+7, -28), Math.toRadians(0),
//                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence purple2 = drive.trajectorySequenceBuilder(startPose)
                // Purple Pixel
                .lineToConstantHeading(new Vector2d(-35, -34))
                .build();

        TrajectorySequence yellow2 = drive.trajectorySequenceBuilder(new Pose2d(-35, -36, Math.toRadians(270)))
                .lineToConstantHeading(new Vector2d(-38, -58))
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(55, -58))

//                .lineToConstantHeading(new Vector2d(30, -60))
//                .splineToConstantHeading(new Vector2d(backboardY-5, -33), Math.toRadians(0))
//                .lineToConstantHeading(new Vector2d(backboardY+10, -33),
//                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence purple3 = drive.trajectorySequenceBuilder(startPose)
                // Purple Pixel
                .lineToSplineHeading(new Pose2d(-39, -40, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-34, -29), Math.toRadians(0))
                .build();
        TrajectorySequence yellow3 = drive.trajectorySequenceBuilder(new Pose2d(-34, -29, Math.toRadians(0)))
                // Yellow Pixel
                .back(3)
                .lineToConstantHeading(new Vector2d(-39, -58))
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(59, -50))

//                .lineToConstantHeading(new Vector2d(30, -55))
//                .splineToConstantHeading(new Vector2d(backboardY-5, -31.5), Math.toRadians(0))
//                .lineToConstantHeading(new Vector2d(backboardY+10, -31.5),
//                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        // Build all potential Trajectory Sequences
        TrajectorySequence path1Purple = drive.trajectorySequenceBuilder(startPose)
                // Purple Pixel
                .lineToSplineHeading(new Pose2d(-37.5, -40, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(-39, -29), Math.toRadians(0))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Start Intake
                    intake();
                })
                .waitSeconds(1)
                .forward(5)
                .strafeRight(9)
                .build();

        TrajectorySequence path2Purple = drive.trajectorySequenceBuilder(startPose)
                // Purple Pixel
                .UNSTABLE_addTemporalMarkerOffset(1.6, () -> {
                    // Start Intake
                    intake();
                })
                .lineToConstantHeading(new Vector2d(-35, -33.75))
                .waitSeconds(1)
                .forward(8)
                .build();

        TrajectorySequence path3Purple = drive.trajectorySequenceBuilder(startPose)
                // Purple Pixel
                .lineToLinearHeading(new Pose2d(-39, -36, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(-36, -29, Math.toRadians(180)))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Start Intake
                    intake();
                })
                .waitSeconds(1)
                .forward(5)
                .strafeLeft(9)
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
                drive.followTrajectorySequence(yellow1);


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
                    drive.followTrajectorySequence(yellow2);
                    sleep(100);
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
                    drive.followTrajectorySequence(yellow3);
                    sleep(100);
                }


            }else{
                drive.followTrajectorySequence(path3Purple);
            }
        }

    }
}
