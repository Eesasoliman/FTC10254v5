package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.resources.DriveOpMode;

@Autonomous

public class TestYellowPixel extends DriveOpMode {
    double lift = 5;
    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);
        drive = initDriveOpMode();
        waitForStart();
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(new Pose2d())
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            prepareScoring(lift);
                        })
                .forward(12)
                .build());
        scorePixelsOnBackboard(lift);
    }
}