package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.resources.ColorPipeline;
import org.firstinspires.ftc.teamcode.resources.DriveOpMode;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Close/Truss 2+X", group = "RED/BLUE")
@Disabled
public class Template extends DriveOpMode {
    int imageNum;
    double parkY;
    double liftHeight = 7.8;
    double dropdownHeight = 0;
    double backboardX = 50;
    double backboardVel = DriveConstants.MAX_VEL / 4;
    double backboardAcc = DriveConstants.MAX_ACCEL / 2;
    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);
        SampleMecanumDrive drive = initDriveOpMode();

        boolean[] driveVariables = initWithController(true);
        if (driveVariables[3]) {
            parkY = 8;
        } else {
            parkY = 60;
        }

        // Build all potential Trajectory Sequences
        TrajectorySequence purple1 = drive.trajectorySequenceBuilder(startPose)

                .build();
        TrajectorySequence purple2 = drive.trajectorySequenceBuilder(startPose)
                .build();
        TrajectorySequence purple3 = drive.trajectorySequenceBuilder(startPose)
                .build();
        TrajectorySequence yellow1 = drive.trajectorySequenceBuilder(purple1.end())
                .build();
        TrajectorySequence yellow2 = drive.trajectorySequenceBuilder(purple2.end())
                .build();
        TrajectorySequence yellow3 = drive.trajectorySequenceBuilder(purple3.end())
                .build();
        TrajectorySequence white1 = drive.trajectorySequenceBuilder(yellow1.end())
                .build();
        TrajectorySequence white2 = drive.trajectorySequenceBuilder(yellow2.end())
                .build();
        TrajectorySequence white3 = drive.trajectorySequenceBuilder(yellow3.end())
                .build();
        TrajectorySequence park1 = drive.trajectorySequenceBuilder(white1.end())
                .build();
        TrajectorySequence park2 = drive.trajectorySequenceBuilder(white2.end())
                .build();
        TrajectorySequence park3 = drive.trajectorySequenceBuilder(white3.end())
                .build();

        ColorPipeline pipeline = startCameras(true);
        while (!isStopRequested() && opModeInInit()) {
            sleep(500);
            imageNum = pipeline.getImageNum();
            telemetry.addData("imageNum", imageNum);
            telemetry.update();
        }

        waitForStart();
        drive.setPoseEstimate(startPose);

        TrajectorySequence purple = null;
        TrajectorySequence yellow = null;
        TrajectorySequence white = null;
        TrajectorySequence park = null;

        if (imageNum == 1) {
            purple = purple1;
            yellow = yellow1;
            white = white1;
            park = park1;
        } else if (imageNum == 2 || imageNum == 3 || imageNum == 4) {
            purple = purple2;
            yellow = yellow2;
            white = white2;
            park = park2;
        } else if (imageNum == 5) {
            purple = purple3;
            yellow = yellow3;
            white = white3;
            park = park3;
        }

        drive.followTrajectorySequence(purple);
        if (driveVariables[0]) {
            drive.followTrajectorySequence(yellow);
//            drive.followTrajectorySequence(relocalize(yellow.end(), 0)); // Start AprilTagCamera 3 seconds before this point
            if (driveVariables[2]) {
                // Follow White Twice
                drive.followTrajectorySequence(white);
//                drive.followTrajectorySequence(relocalize(white.end(), 0)); // Start AprilTagCamera 3 seconds before this point
                drive.followTrajectorySequence(white);
            } else if (driveVariables[1]) {
                // Follow White Once
                drive.followTrajectorySequence(white);
            }
        }
        drive.followTrajectorySequence(park);
    }
}
