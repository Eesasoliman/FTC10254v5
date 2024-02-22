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

public class BLUEback extends DriveOpMode {
    int imageNum;
    double  liftHeight = 5;
    double backboardY = 50.5;

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        boolean[] driveVariables = initWithController(false);
        double parkY=0;
        if (driveVariables[1]){
            parkY=12;
        }else{
            parkY=60;
        }
        double backVel = DriveConstants.MAX_VEL/4;

        // Start Positions for Different Trajectories
        Pose2d startPose = new Pose2d(15.5, 63, Math.toRadians(90));

        TrajectorySequence path1 = drive.trajectorySequenceBuilder(startPose)
                //purple pixel
                .lineToSplineHeading(new Pose2d(30, 45, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(37,28), Math.toRadians(270))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intake();
                })
                .waitSeconds(1)
                .forward(5)

                //yellow pixel
//                .UNSTABLE_addTemporalMarkerOffset(0.65,() -> {
                .UNSTABLE_addTemporalMarkerOffset(0.2,() -> {
                    prepareScoring(liftHeight);
                })
                .lineToConstantHeading(new Vector2d(backboardY-5,41.5))
                .lineToSplineHeading(
        new Pose2d(backboardY+10, 41.5, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0.2,() -> {
                    scorePixelsOnBackboard(liftHeight);
                })
                .waitSeconds(1)
                .back(12,SampleMecanumDrive.getVelocityConstraint(backVel*4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .lineToConstantHeading(new Vector2d(52,parkY))
                .forward(5)
                .build();

        TrajectorySequence path2 = drive.trajectorySequenceBuilder(startPose)
                //purple pixel
                .lineToConstantHeading(new Vector2d(14, 34.5))
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> {
                    intake();
                })
                .waitSeconds(0.5)
                .forward(5)

                //yellow pixel
//                .UNSTABLE_addTemporalMarkerOffset(1.5,() -> {
                .UNSTABLE_addTemporalMarkerOffset(.5,() -> {
                    prepareScoring(liftHeight);
                })
                .lineToLinearHeading(new Pose2d(backboardY-5, 35.5, Math.toRadians(0)))
                .waitSeconds(0.5)
                .lineToSplineHeading(
                        new Pose2d(backboardY+10,35.5, -90),
                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    scorePixelsOnBackboard(liftHeight);
                })
                .waitSeconds(1)
                .back(12,SampleMecanumDrive.getVelocityConstraint(backVel*4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .lineToConstantHeading(new Vector2d(52,parkY))
                .forward(10)
                .build();

        TrajectorySequence path3 = drive.trajectorySequenceBuilder(startPose)
                //Purple pixel
                .lineToConstantHeading(new Vector2d(17,38))
                .lineToSplineHeading( new Pose2d(14.5, 34, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intake();
                })
                .waitSeconds(0.5)
                .forward(10)

                //Yellow pixel
//                .UNSTABLE_addTemporalMarkerOffset(1.2,() -> {
                .UNSTABLE_addTemporalMarkerOffset(.2,() -> {
                    prepareScoring(liftHeight);
                })
                .lineToSplineHeading( new Pose2d(backboardY-5, 28.5, Math.toRadians(0)))
                .lineToSplineHeading(
                        new Pose2d(backboardY+10, 28.5, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0.2,() -> {
                    scorePixelsOnBackboard(liftHeight);
                })
                .waitSeconds(1)
                .back(12,SampleMecanumDrive.getVelocityConstraint(backVel*4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .lineToConstantHeading(new Vector2d(52,parkY))
                .forward(10)
                .build();

        TrajectorySequence whitepath1inner = drive.trajectorySequenceBuilder(startPose)
                //purple pixel
                .UNSTABLE_addTemporalMarkerOffset(2.46, () -> {
                    intake();
                })
                .lineToSplineHeading(new Pose2d(30, 45, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(37,28), Math.toRadians(270))
                .waitSeconds(1)
                .forward(5)

                //yellow pixel
                .UNSTABLE_addTemporalMarkerOffset(0.65,() -> {
                    prepareScoring(liftHeight);
                })
                .lineToConstantHeading(new Vector2d(backboardY-5,41.5))
                .lineToSplineHeading(
                        new Pose2d(backboardY+10, 41.5, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0.2,() -> {
                    scorePixelsOnBackboard(liftHeight);
                })
                .waitSeconds(1)
                .back(12,SampleMecanumDrive.getVelocityConstraint(backVel*4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                // White Pixel
                // Pick up
                .splineToSplineHeading(new Pose2d(48, 12, Math.toRadians(0)), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-48, 12, Math.toRadians(0)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-55, 14, Math.toRadians(0)), Math.toRadians(-90))
                .forward(6,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.FPS.setPosition(0.4);
                    robot.IN.setPower(1);
                    sleep(3000);
                    robot.FPS.setPosition(0);
                    robot.IN.setPower(-1);
                    sleep(1000);
                    robot.IN.setPower(0);
                })
                .waitSeconds(3)
                // Score white
                .splineToSplineHeading(new Pose2d(-48, 12, Math.toRadians(0)), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(48, 12, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(backboardY-5, 28.5, Math.toRadians(0)), Math.toRadians(-90))
                .splineToSplineHeading(
                        new Pose2d(backboardY + 10, 28.5),
                        Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .lineToConstantHeading(new Vector2d(52,parkY))
                .forward(5)
                .build();

        TrajectorySequence whitepath2inner = drive.trajectorySequenceBuilder(startPose)
                //purple pixel
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> {
                    intake();
                })
                .lineToConstantHeading(new Vector2d(14, 34.5))
                .waitSeconds(0.5)
                .forward(5)

                //yellow pixel
                .UNSTABLE_addTemporalMarkerOffset(1.5,() -> {
                    prepareScoring(liftHeight);
                })
                .lineToSplineHeading(new Pose2d(backboardY-5, 35, Math.toRadians(0)))
                .lineToLinearHeading(
                        new Pose2d(backboardY+10,35.5,Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    scorePixelsOnBackboard(liftHeight);
                })
                .waitSeconds(1)
                .back(12,SampleMecanumDrive.getVelocityConstraint(backVel*4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                // White Pixel
                // Pick up
                .splineToSplineHeading(new Pose2d(48, 12, Math.toRadians(0)), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-48, 12, Math.toRadians(0)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-55, 24, Math.toRadians(0)), Math.toRadians(-90))
                .forward(6,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.FPS.setPosition(0.4);
                    robot.IN.setPower(1);
                    sleep(3000);
                    robot.FPS.setPosition(0);
                    robot.IN.setPower(-1);
                    sleep(1000);
                    robot.IN.setPower(0);
                })
                .waitSeconds(3)
                // Score white
                .splineToSplineHeading(new Pose2d(-48, 12, Math.toRadians(0)), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(48, 12, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(backboardY-5, 28.5, Math.toRadians(0)), Math.toRadians(-90))
                .splineToSplineHeading(
                        new Pose2d(backboardY + 10, 28.5),
                        Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .lineToConstantHeading(new Vector2d(52,parkY))
                .forward(10)
                .build();

        TrajectorySequence whitepath3inner = drive.trajectorySequenceBuilder(startPose)
                //Purple pixel
                .UNSTABLE_addTemporalMarkerOffset(2.07, () -> {
                    intake();
                })
                .lineToConstantHeading(new Vector2d(17,38))
                .lineToSplineHeading( new Pose2d(14.5, 34, Math.toRadians(0)))
                .waitSeconds(0.5)
                .forward(10)

                //Yellow pixel
                .UNSTABLE_addTemporalMarkerOffset(1.2,() -> {
                    prepareScoring(liftHeight);
                })
                .lineToSplineHeading( new Pose2d(backboardY-5, 28.5, Math.toRadians(0)))
                .lineToSplineHeading(
                        new Pose2d(backboardY+10, 28.5, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0.2,() -> {
                    scorePixelsOnBackboard(liftHeight);
                })
                .waitSeconds(1)
                .back(12,SampleMecanumDrive.getVelocityConstraint(backVel*4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                // White Pixel
                // Pick up
                .splineToSplineHeading(new Pose2d(48, 12, Math.toRadians(0)), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-48, 12, Math.toRadians(0)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-55, 34, Math.toRadians(0)), Math.toRadians(-90))
                .forward(6,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.FPS.setPosition(0.4);
                    robot.IN.setPower(1);
                    sleep(3000);
                    robot.FPS.setPosition(0);
                    robot.IN.setPower(-1);
                    sleep(1000);
                    robot.IN.setPower(0);
                })
                .waitSeconds(3)
                // Score white
                .splineToSplineHeading(new Pose2d(-48, 12, Math.toRadians(0)), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(48, 12, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(backboardY-5, 28.5, Math.toRadians(0)), Math.toRadians(-90))
                .splineToSplineHeading(
                        new Pose2d(backboardY + 10, 28.5),
                        Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .lineToConstantHeading(new Vector2d(52,parkY))
                .forward(10)
                .build();

        TrajectorySequence whitepath1outer = drive.trajectorySequenceBuilder(startPose)
                //purple pixel
                .UNSTABLE_addTemporalMarkerOffset(2.46, () -> {
                    intake();
                })
                .lineToSplineHeading(new Pose2d(30, 45, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(37,28), Math.toRadians(270))
                .waitSeconds(1)
                .forward(5)

                //yellow pixel
                .UNSTABLE_addTemporalMarkerOffset(0.65,() -> {
                    prepareScoring(liftHeight);
                })
                .lineToConstantHeading(new Vector2d(backboardY-5,41.5))
                .lineToSplineHeading(
                        new Pose2d(backboardY+10, 41.5, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0.2,() -> {
                    scorePixelsOnBackboard(liftHeight);
                })
                .waitSeconds(1)
                .back(12,SampleMecanumDrive.getVelocityConstraint(backVel*4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                // White Pixel
                // Pick up
                .splineToSplineHeading(new Pose2d(48, 60, Math.toRadians(0)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-48, 60, Math.toRadians(0)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-55, 14, Math.toRadians(0)), Math.toRadians(90))
                .forward(6,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.FPS.setPosition(0.4);
                    robot.IN.setPower(1);
                    sleep(3000);
                    robot.FPS.setPosition(0);
                    robot.IN.setPower(-1);
                    sleep(1000);
                    robot.IN.setPower(0);
                })
                .waitSeconds(3)
                // Score white
                .splineToSplineHeading(new Pose2d(-48, 60, Math.toRadians(0)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(48, 60, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(backboardY-5, 28.5, Math.toRadians(0)), Math.toRadians(90))
                .splineToSplineHeading(
                        new Pose2d(backboardY + 10, 28.5),
                        Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .lineToConstantHeading(new Vector2d(52,parkY))
                .forward(5)
                .build();

        TrajectorySequence whitepath2outer = drive.trajectorySequenceBuilder(startPose)
                //purple pixel
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> {
                    intake();
                })
                .lineToConstantHeading(new Vector2d(14, 34.5))
                .waitSeconds(0.5)
                .forward(5)

                //yellow pixel
                .UNSTABLE_addTemporalMarkerOffset(1.5,() -> {
                    prepareScoring(liftHeight);
                })
                .lineToSplineHeading(new Pose2d(backboardY-5, 35, Math.toRadians(0)))
                .lineToLinearHeading(
                        new Pose2d(backboardY+10,35.5,Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    scorePixelsOnBackboard(liftHeight);
                })
                .waitSeconds(1)
                .back(12,SampleMecanumDrive.getVelocityConstraint(backVel*4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                // White Pixel
                // Pick up
                .splineToSplineHeading(new Pose2d(48, 60, Math.toRadians(0)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-48, 60, Math.toRadians(0)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-55, 24, Math.toRadians(0)), Math.toRadians(90))
                .forward(6,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.FPS.setPosition(0.4);
                    robot.IN.setPower(1);
                    sleep(3000);
                    robot.FPS.setPosition(0);
                    robot.IN.setPower(-1);
                    sleep(1000);
                    robot.IN.setPower(0);
                })
                .waitSeconds(3)
                // Score white
                .splineToSplineHeading(new Pose2d(-48, 60, Math.toRadians(0)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(48, 60, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(backboardY-5, 28.5, Math.toRadians(0)), Math.toRadians(90))
                .splineToSplineHeading(
                        new Pose2d(backboardY + 10, 28.5),
                        Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .lineToConstantHeading(new Vector2d(52,parkY))
                .forward(10)
                .build();

        TrajectorySequence whitepath3outer = drive.trajectorySequenceBuilder(startPose)
                //Purple pixel
                .UNSTABLE_addTemporalMarkerOffset(2.07, () -> {
                    intake();
                })
                .lineToConstantHeading(new Vector2d(17,38))
                .lineToSplineHeading( new Pose2d(14.5, 34, Math.toRadians(0)))
                .waitSeconds(0.5)
                .forward(10)

                //Yellow pixel
                .UNSTABLE_addTemporalMarkerOffset(1.2,() -> {
                    prepareScoring(liftHeight);
                })
                .lineToSplineHeading( new Pose2d(backboardY-5, 28.5, Math.toRadians(0)))
                .lineToSplineHeading(
                        new Pose2d(backboardY+10, 28.5, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(backVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0.2,() -> {
                    scorePixelsOnBackboard(liftHeight);
                })
                .waitSeconds(1)
                .back(12,SampleMecanumDrive.getVelocityConstraint(backVel*4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                // White Pixel
                // Pick up
                .splineToSplineHeading(new Pose2d(48, 60, Math.toRadians(0)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-48, 60, Math.toRadians(0)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-55, 34, Math.toRadians(0)), Math.toRadians(90))
                .forward(6,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.FPS.setPosition(0.4);
                    robot.IN.setPower(1);
                    sleep(3000);
                    robot.FPS.setPosition(0);
                    robot.IN.setPower(-1);
                    sleep(1000);
                    robot.IN.setPower(0);
                })
                .waitSeconds(3)
                // Score white
                .splineToSplineHeading(new Pose2d(-48, 60, Math.toRadians(0)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(48, 60, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(backboardY-5, 28.5, Math.toRadians(0)), Math.toRadians(90))
                .splineToSplineHeading(
                        new Pose2d(backboardY + 10, 28.5),
                        Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .lineToConstantHeading(new Vector2d(52,parkY))
                .forward(10)
                .build();

        TrajectorySequence path1Purple = drive.trajectorySequenceBuilder(startPose)
                //purple pixel
                .UNSTABLE_addTemporalMarkerOffset(2.46, () -> {
                    intake();
                })
                .lineToSplineHeading(new Pose2d(30, 45, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(37,28), Math.toRadians(270))
                .waitSeconds(1)
                .forward(5)
                .build();

        TrajectorySequence path2Purple = drive.trajectorySequenceBuilder(startPose)
                //purple pixel
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> {
                    intake();
                })
                .lineToConstantHeading(new Vector2d(14, 34.5))
                .waitSeconds(0.5)
                .forward(5)
                .build();

        TrajectorySequence path3Purple = drive.trajectorySequenceBuilder(startPose)
                //Purple pixel
                .UNSTABLE_addTemporalMarkerOffset(2.07, () -> {
                    intake();
                })
                .lineToConstantHeading(new Vector2d(17,38))
                .lineToSplineHeading( new Pose2d(14.5, 34, Math.toRadians(0)))
                .waitSeconds(0.5)
                .forward(10)
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
 //purple

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
