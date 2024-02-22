package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.resources.DriveOpMode;
import org.firstinspires.ftc.teamcode.resources.BlueColorPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous

public class BLUEfront extends DriveOpMode {
    int imageNum;

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);
        SampleMecanumDrive drive = initDriveOpMode();

        boolean[] driveVariables = initWithController(true);
        double liftHeight = 5;
        double parkY=0;
        double backboardY = 50;
        if (driveVariables[1]){
            parkY=10;
        }else{
            parkY=60;
        }
        double backVel = DriveConstants.MAX_VEL/4;

        Pose2d startPose = new Pose2d(-37.5, 63, Math.toRadians(90));
        // Build all potential Trajectory Sequences
        TrajectorySequence path1 = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    sleep(1000);
                    // Start Intake
                    intake();
                    sleep(5000);
                    prepareScoring(liftHeight);
                    sleep(3000);
                    scorePixelsOnBackboard(liftHeight);
                })
                // Purple Pixel
                .lineToSplineHeading(new Pose2d(-37.5, 40, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-34, 29), Math.toRadians(0))
                .waitSeconds(0.5)
                .forward(7)

                // Yellow Pixel
                .lineToConstantHeading(new Vector2d(-39, 8))
                .turn(Math.toRadians(180))
//                .UNSTABLE_addTemporalMarkerOffset(.8, () -> {
//
//                    // Prepare Lift
//                    prepareScoring(liftHeight);
//                })
                .lineToConstantHeading(new Vector2d(30, 8))
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(backboardY-5, 37), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(backboardY+10, 37),
                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    // Drop Pixels
//                    scorePixelsOnBackboard(liftHeight);
//                })
                .waitSeconds(1)

                // Park
                .back(12)
                .lineToConstantHeading(new Vector2d(52,parkY))
                .forward(10)
                .build();

        TrajectorySequence path2 = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(4.5, () -> {
                    // Start Intake
                    intake();
                })
                .addTemporalMarker(9, () ->
                {
                    prepareScoring(liftHeight);
                })
                .addTemporalMarker(1, () -> {
                    scorePixelsOnBackboard(liftHeight);
                })
                // Purple Pixel
                .lineToConstantHeading(new Vector2d(-37, 12))
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL/4, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .turn(Math.toRadians(180))
                .back(2)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    // Start Intake
                    intake();
                })
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(1)
                .forward(8)

                // Yellow Pixel
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(30, 12))
//                .UNSTABLE_addTemporalMarkerOffset(.2, () -> {
//                    // Prepare Lift
//                    prepareScoring(liftHeight);
//                })
                .splineToConstantHeading(new Vector2d(backboardY-5, 35), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(backboardY+10, 35),
                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    // Drop Pixels
//                    scorePixelsOnBackboard(liftHeight);
//                })
                .waitSeconds(1)

                // Park
                .back(12)
                .lineToConstantHeading(new Vector2d(52,parkY))
                .forward(10)
                .build();

        TrajectorySequence path3 = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(3.9, () -> {
                    // Start Intake
                    intake();
                })
                .addTemporalMarker(9, () ->
                {
                    prepareScoring(liftHeight);
                })
                .addTemporalMarker(1, () -> {
                    scorePixelsOnBackboard(liftHeight);
                })
                // Purple Pixel
                .strafeLeft(3)
                .lineToLinearHeading(new Pose2d(-38, 29, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Start Intake
                    intake();
                })
                .waitSeconds(0.5)
                .forward(2)
                .lineToConstantHeading(new Vector2d(-38, 8))//change

                // Yellow Pixel
                .lineToConstantHeading(new Vector2d(32, 12))
//                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
//                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
//                    // Prepare Lift
//                    prepareScoring(liftHeight);
//                })
                .lineToConstantHeading(new Vector2d(backboardY-5, 26))
                .splineToConstantHeading(new Vector2d(backboardY+7, 28), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    // Drop Pixels
//                    scorePixelsOnBackboard(liftHeight);
//                })
                .waitSeconds(1)

                // Park
                .back(12)
                .lineToConstantHeading(new Vector2d(52,parkY))
                .forward(10)
                .build();


        TrajectorySequence whitepath1inner = drive.trajectorySequenceBuilder(startPose)
                // Purple Pixel
                .UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    // Start Intake
                    intake();
                })
                .lineToLinearHeading(new Pose2d(-38, 29, Math.toRadians(0)))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(-35, 12))

                // White Pixel
                .strafeRight(1)
                .splineToConstantHeading(new Vector2d(-61, 12), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.IN.setPower(1);
                })
                .forward(6,
                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift(true, 2);
                    robot.IN.setPower(-1);
                    sleep(1000);
                    robot.IN.setPower(0);
                })

                // Yellow Pixel
                .lineToConstantHeading(new Vector2d(30, 12))
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    // Prepare Lift
                    prepareScoring(liftHeight);
                })
                .lineToConstantHeading(new Vector2d(backboardY-5, -28.5))
                .lineToConstantHeading(new Vector2d(backboardY+10, -28.5),
                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Drop Pixels
                    scorePixelsOnBackboard(liftHeight);
                })
                .waitSeconds(1)
                .back(12)
                .lineToConstantHeading(new Vector2d(52,parkY))
                .forward(10)
                .build();

        TrajectorySequence whitepath2inner = drive.trajectorySequenceBuilder(startPose)
                // Purple Pixel
                .lineToConstantHeading(new Vector2d(-35, 12))
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL/4, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .turn(Math.toRadians(180))
                .back(2)
                .UNSTABLE_addTemporalMarkerOffset(0.01, () -> {
                    // Start Intake
                    intake();
                })
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(0.5)
                .forward(3)

                // White Pixel
                .splineToLinearHeading(new Pose2d(-61, 12, Math.toRadians(0)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.IN.setPower(1);
                })
                .forward(6,
                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift(true, 2);
                    robot.IN.setPower(-1);
                    sleep(1000);
                    robot.IN.setPower(0);
                })

                // Yellow Pixel
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(30, 12))
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    // Prepare Lift
                    prepareScoring(liftHeight);
                })
                .splineToConstantHeading(new Vector2d(backboardY-5, 33), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(backboardY+10, 33),
                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Drop Pixels
                    scorePixelsOnBackboard(liftHeight);
                })
                .waitSeconds(1)

                .back(12)
                .lineToConstantHeading(new Vector2d(52,parkY))
                .forward(10)
                .build();

        TrajectorySequence whitepath3inner = drive.trajectorySequenceBuilder(startPose)
                // Purple Pixel
                .UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    // Start Intake
                    intake();
                })
                .lineToSplineHeading(new Pose2d(-37.5, 40, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-34, 29), Math.toRadians(0))
                .waitSeconds(0.5)

                // White Pixel
                .strafeLeft(1)
                .splineToLinearHeading(new Pose2d(-61, -12, Math.toRadians(0)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.IN.setPower(1);
                })
                .forward(6,
                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift(true, 2);
                    robot.IN.setPower(-1);
                    sleep(1000);
                    robot.IN.setPower(0);
                })
                .back(2)

                // Yellow Pixel
                .lineToConstantHeading(new Vector2d(-35, 12))
                .turn(Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(30, 12))
                .UNSTABLE_addTemporalMarkerOffset(1.10, () -> {
                    // Prepare Lift
                    prepareScoring(liftHeight);
                })
                .splineToConstantHeading(new Vector2d(backboardY-5, -41.5), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(backboardY+10, -41.5),
                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Drop Pixels
                    scorePixelsOnBackboard(liftHeight);
                })
                .waitSeconds(1)

                .back(12)
                .lineToConstantHeading(new Vector2d(52,parkY))
                .forward(10)
                .build();

        TrajectorySequence whitepath1outer = drive.trajectorySequenceBuilder(startPose)
                // Purple Pixel
                .UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    // Start Intake
                    intake();
                })
                .lineToLinearHeading(new Pose2d(-38, 29, Math.toRadians(0)))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(-35, 12))

                // White Pixel
                .strafeRight(1)
                .splineToConstantHeading(new Vector2d(-61, 12), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.IN.setPower(1);
                })
                .forward(6,
                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift(true, 2);
                    robot.IN.setPower(-1);
                    sleep(1000);
                    robot.IN.setPower(0);
                })
                .back(2)

                // Yellow Pixel
                .splineToConstantHeading(new Vector2d(-36, 60), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(30, 60))
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    // Prepare Lift
                    prepareScoring(liftHeight);
                })
                .lineToConstantHeading(new Vector2d(backboardY-5, 28.5))
                .lineToConstantHeading(new Vector2d(backboardY+10, 28.5),
                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Drop Pixels
                    scorePixelsOnBackboard(liftHeight);
                })
                .waitSeconds(1)
                .back(12)
                .lineToConstantHeading(new Vector2d(52,parkY))
                .forward(10)
                .build();

        TrajectorySequence whitepath2outer = drive.trajectorySequenceBuilder(startPose)
                // Purple Pixel
                .lineToConstantHeading(new Vector2d(-35, 12))
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL/4, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .turn(Math.toRadians(180))
                .back(2)
                .UNSTABLE_addTemporalMarkerOffset(0.01, () -> {
                    // Start Intake
                    intake();
                })
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(0.5)
                .forward(3)

                // White Pixel
                .splineToLinearHeading(new Pose2d(-61, 12, Math.toRadians(0)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.IN.setPower(1);
                })
                .forward(6,
                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift(true, 2);
                    robot.IN.setPower(-1);
                    sleep(1000);
                    robot.IN.setPower(0);
                })
                .back(2)

                // Yellow Pixel
                .splineToConstantHeading(new Vector2d(-36, 60), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(30, 60))
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    // Prepare Lift
                    prepareScoring(liftHeight);
                })
                .splineToConstantHeading(new Vector2d(backboardY-5, 33), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(backboardY+10, 33),
                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Drop Pixels
                    scorePixelsOnBackboard(liftHeight);
                })
                .waitSeconds(1)

                .back(12)
                .lineToConstantHeading(new Vector2d(52,parkY))
                .forward(10)
                .build();

        TrajectorySequence whitepath3outer = drive.trajectorySequenceBuilder(startPose)
                // Purple Pixel
                .UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    // Start Intake
                    intake();
                })
                .lineToSplineHeading(new Pose2d(-37.5, 40, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-34, 29), Math.toRadians(0))
                .waitSeconds(0.5)

                // White Pixel
                .strafeLeft(1)
                .splineToLinearHeading(new Pose2d(-61, -12, Math.toRadians(0)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.IN.setPower(1);
                })
                .forward(6,
                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift(true, 2);
                    robot.IN.setPower(-1);
                    sleep(1000);
                    robot.IN.setPower(0);
                })
                .back(2)

                // Yellow Pixel
                .splineToConstantHeading(new Vector2d(-36, 60), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(30, 60))
                .UNSTABLE_addTemporalMarkerOffset(1.10, () -> {
                    // Prepare Lift
                    prepareScoring(liftHeight);
                })
                .splineToConstantHeading(new Vector2d(backboardY-5, 41.5), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(backboardY+10, 41.5),
                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Drop Pixels
                    scorePixelsOnBackboard(liftHeight);
                })
                .waitSeconds(1)

                .back(12)
                .lineToConstantHeading(new Vector2d(52,parkY))
                .forward(10)
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
                .UNSTABLE_addTemporalMarkerOffset(1.6, () -> {intake();})
                .lineToConstantHeading(new Vector2d(-36, 34))
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

            //yellow
            if(driveVariables[0])
            {
                if (!driveVariables[2]) {
                    //yellow
                    drive.followTrajectorySequence(path1);
                }
                else {
                    //white
                    if (driveVariables[3]) {
                        drive.followTrajectorySequence(whitepath1inner);
                    }
                    else
                    {
                        drive.followTrajectorySequence(whitepath1outer);
                    }
                }

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
                    drive.followTrajectorySequence(path2);
                }
                else {
                    //white
                    if (driveVariables[3]) {
                        drive.followTrajectorySequence(whitepath2inner);
                    }
                    else
                    {
                        drive.followTrajectorySequence(whitepath2outer);
                    }
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
                    drive.followTrajectorySequence(path3);
                }
                else {
                    //white
                    if (driveVariables[3]) {
                        drive.followTrajectorySequence(whitepath3inner);
                    }
                    else
                    {
                        drive.followTrajectorySequence(whitepath3outer);
                    }
                }

            }else{
                drive.followTrajectorySequence(path3Purple);
            }
        }
    }

}
