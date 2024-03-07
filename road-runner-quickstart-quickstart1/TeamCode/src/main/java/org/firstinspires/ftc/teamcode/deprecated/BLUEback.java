package org.firstinspires.ftc.teamcode.deprecated;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.resources.BlueColorPipeline;
import org.firstinspires.ftc.teamcode.resources.DriveOpMode;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Disabled
public class BLUEback extends DriveOpMode {
    int imageNum;
    double  liftHeight = 5.5;
    double backboardY = 51.5;


    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        boolean[] driveVariables = initWithController(false);
        double parkY = 0;
        double parkX = 49;
        if (driveVariables[1])
        {
            parkY = 12;
        }
        else {
            parkY = 60;
        }
        double backVel = DriveConstants.MAX_VEL/4;

        // Start Positions for Different Trajectories
        Pose2d startPose = new Pose2d(15.5, 63, Math.toRadians(90));

        TrajectorySequence path1 = drive.trajectorySequenceBuilder(startPose)
                //purple pixel
                .lineToSplineHeading(new Pose2d(30, 45, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(37,28), Math.toRadians(270))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    purpleIntake();
                })
                .waitSeconds(1)
                .forward(5)

                // Yellow
                .lineToConstantHeading(new Vector2d(backboardY-5,41.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    prepareScoring(liftHeight);
                })
                .lineToLinearHeading(
                        new Pose2d(backboardY+10, 43.5, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    scorePixelsOnBackboard(liftHeight);
                })
                .waitSeconds(1)
                .back(12)
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> {
                    resetForTeleOp(liftHeight);
                })

                // Park
                .lineToConstantHeading(new Vector2d(parkX,parkY))
                .forward(5)
                .build();

        TrajectorySequence path2 = drive.trajectorySequenceBuilder(startPose)
                //purple pixel
                .lineToConstantHeading(new Vector2d(14, 33.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    purpleIntake();
                })
                .waitSeconds(0.5)
                .forward(5)

                // Yellow Pixel
                .UNSTABLE_addTemporalMarkerOffset(.5,() -> {
                    prepareScoring(liftHeight);
                })
                .lineToLinearHeading(new Pose2d(backboardY-5, 35.5, Math.toRadians(0)))
                .lineToConstantHeading(
                        new Vector2d(backboardY+10,35.5),
                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(.2)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    scorePixelsOnBackboard(liftHeight);
                })
                .waitSeconds(1)
                .back(12,SampleMecanumDrive.getVelocityConstraint(backVel*4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> {
                    resetForTeleOp(liftHeight);
                })

                // Park
                .lineToConstantHeading(new Vector2d(parkX,parkY))
                .forward(10)
                .build();

        TrajectorySequence path3 = drive.trajectorySequenceBuilder(startPose)
                //Purple pixel
                .lineToConstantHeading(new Vector2d(17,38))
                .lineToSplineHeading( new Pose2d(14.5, 34, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    purpleIntake();
                })
                .waitSeconds(0.5)
                .forward(10)

                // Yellow Pixel
                .UNSTABLE_addTemporalMarkerOffset(.2,() -> {
                    prepareScoring(liftHeight);
                })
                .lineToSplineHeading( new Pose2d(backboardY-5, 28.5, Math.toRadians(0)))
                .lineToSplineHeading(
                        new Pose2d(backboardY+10, 27.5, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0.2,() -> {
                    scorePixelsOnBackboard(liftHeight);
                })
                .waitSeconds(1)
                .back(12,SampleMecanumDrive.getVelocityConstraint(backVel*4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> {
                    resetForTeleOp(liftHeight);
                })

                // Park
                .lineToConstantHeading(new Vector2d(parkX,parkY))
                .forward(10)
                .build();

        TrajectorySequence path1Purple = drive.trajectorySequenceBuilder(startPose)
                //purple pixel
                .lineToSplineHeading(new Pose2d(30, 45, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(37,28), Math.toRadians(270))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    purpleIntake();
                })
                .waitSeconds(1)
                .forward(5)
                .build();

        TrajectorySequence path2Purple = drive.trajectorySequenceBuilder(startPose)
                //purple pixel
                .lineToConstantHeading(new Vector2d(14, 34.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    purpleIntake();
                })
                .waitSeconds(1)
                .forward(5)
                .build();

        TrajectorySequence path3Purple = drive.trajectorySequenceBuilder(startPose)
                //Purple pixel
                .lineToConstantHeading(new Vector2d(17,38))
                .lineToSplineHeading( new Pose2d(14.5, 34, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    purpleIntake();
                })
                .waitSeconds(1)
                .forward(10)
                .build();

        BlueColorPipeline pipeline = startBlueCamera();
        while (!isStopRequested() && !opModeIsActive())
        {
            sleep(1000);
            imageNum = pipeline.getImageNum();
            telemetry.addData("imageNum", imageNum);
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) { return; }
        drive.setPoseEstimate(startPose);

        //purple
        if (imageNum == 1)
        {
            //yellow
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
