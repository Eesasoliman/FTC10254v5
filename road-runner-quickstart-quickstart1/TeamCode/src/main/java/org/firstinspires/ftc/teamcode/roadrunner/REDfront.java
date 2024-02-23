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

public class REDfront extends DriveOpMode {
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
        TrajectorySequence path1 = drive.trajectorySequenceBuilder(startPose)
                // Purple Pixel
                .strafeRight(3)
                .lineToLinearHeading(new Pose2d(-38, -29, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Start Intake
                    intake();
                })
                .waitSeconds(0.5)
                .forward(2)
                .lineToConstantHeading(new Vector2d(-38, -8))//change

                // Yellow Pixel
                .lineToConstantHeading(new Vector2d(32, -12))
//                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {

                    // Prepare Lift
                    prepareScoring(liftHeight);
                })
                .lineToConstantHeading(new Vector2d(backboardY-5, -26))
                .splineToConstantHeading(new Vector2d(backboardY+7, -28), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Drop Pixels
                    scorePixelsOnBackboard(liftHeight);
                })
                .waitSeconds(afterdrop)
                .back(12)
                .lineToConstantHeading(new Vector2d(52,parkY))
                .forward(10)
                .build();

        TrajectorySequence path2 = drive.trajectorySequenceBuilder(startPose)
                // Purple Pixel
                .lineToConstantHeading(new Vector2d(-35, -12))
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
                .turn(Math.toRadians(-90))
                .lineToConstantHeading(new Vector2d(30, -12))
//                .UNSTABLE_addTemporalMarkerOffset(.8, () -> {
                .UNSTABLE_addTemporalMarkerOffset(.2, () -> {
                    // Prepare Lift
                    prepareScoring(liftHeight);
                })
                .splineToConstantHeading(new Vector2d(backboardY-5, -33), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(backboardY+10, -33),
                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Drop Pixels
                    scorePixelsOnBackboard(liftHeight);
                })
                .waitSeconds(afterdrop)

                .back(12)
                .lineToConstantHeading(new Vector2d(52,parkY))
                .forward(10)
                .build();

        TrajectorySequence path3 = drive.trajectorySequenceBuilder(startPose)
                // Purple Pixel
                .lineToSplineHeading(new Pose2d(-37.5, -40, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-34, -29), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Start Intake
                    intake();
                })
                .waitSeconds(0.5)
                .forward(7)

                // Yellow Pixel
                .lineToConstantHeading(new Vector2d(-39, -8))
                .turn(Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(30, -8))
//                .UNSTABLE_addTemporalMarkerOffset(1.10, () -> {
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    // Prepare Lift
                    prepareScoring(liftHeight);
                })
                .splineToConstantHeading(new Vector2d(backboardY-5, -31.5), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(backboardY+10, -31.5),
                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Drop Pixels
                    scorePixelsOnBackboard(liftHeight);
                })
                .waitSeconds(afterdrop)

                .back(12)
                .lineToConstantHeading(new Vector2d(52,parkY))
                .forward(10)
                .build();

        // Build all potential Trajectory Sequences
        TrajectorySequence path1Purple = drive.trajectorySequenceBuilder(startPose)
                // Purple Pixel
                .lineToSplineHeading(new Pose2d(-37.5, -40, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(-38, -29), Math.toRadians(0))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Start Intake
                    intake();
                })
                .waitSeconds(1)
                .forward(5)
                .build();

        TrajectorySequence path2Purple = drive.trajectorySequenceBuilder(startPose)
                // Purple Pixel
                .UNSTABLE_addTemporalMarkerOffset(1.6, () -> {
                    // Start Intake
                    intake();
                })
                .lineToConstantHeading(new Vector2d(-35, -34))
                .waitSeconds(1)
                .forward(5)
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
