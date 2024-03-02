package org.firstinspires.ftc.teamcode.deprecated;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.resources.DriveOpMode;
import org.firstinspires.ftc.teamcode.resources.RedColorPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Disabled
public class TRUSSred extends DriveOpMode {
    int imageNum;

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);
        drive = initDriveOpMode();

        boolean[] driveVariables = initWithController(true);
        double liftHeight = 7.8;
        double backboardY = 50;
        double afterdrop = 1;
        double parkY = 0;
        if (driveVariables[1]){
            parkY = -8;
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
                .splineToConstantHeading(new Vector2d(-48, -36), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    intake();
                })

                // Yellow Pixel
                .lineToLinearHeading(new Pose2d(-37.5, -58))
                .setConstraints(
                    SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .lineToConstantHeading(new Vector2d(24, -58))
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    prepareScoring(liftHeight);
                })
                .lineToConstantHeading(new Vector2d(backboardY-5, -26))
                .splineToConstantHeading(new Vector2d(backboardY+7, -28), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorePixelsOnBackboard(liftHeight);
                })

                // Park
                .waitSeconds(afterdrop)
                .setConstraints(
                    SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .back(12)
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> {
                    resetForTeleOp(liftHeight);
                })
                .lineToConstantHeading(new Vector2d(52,parkY))
                .forward(10)
                .build();

        TrajectorySequence path2 = drive.trajectorySequenceBuilder(startPose)
                // Purple Pixel
                .lineToConstantHeading(new Vector2d(-37, -33.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intake();
                })
                .waitSeconds(0.5)

                // Yellow Pixel
//                .splineToConstantHeading(new Vector2d(-43, -27), 270) use if it hits truss
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
                    scorePixelsOnBackboard(liftHeight);
                })
                .waitSeconds(afterdrop)

                // Park
                .setConstraints(
                    SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .back(12)
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> {
                    resetForTeleOp(liftHeight);
                })
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

                // Yellow Pixel
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
                    scorePixelsOnBackboard(liftHeight);
                })
                .waitSeconds(afterdrop)

                // Park
                .setConstraints(
                    SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .back(12)
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> {
                    resetForTeleOp(liftHeight);
                })
                .lineToConstantHeading(new Vector2d(52, parkY))
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