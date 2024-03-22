package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.resources.ColorPipeline;
import org.firstinspires.ftc.teamcode.resources.DriveOpMode;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class RedTruss extends DriveOpMode {
    int imageNum;
    double parkY;
    double liftHeight = 7;
    double backboardX = 51.5;

    double stackY = -13;
    double slowVel = DriveConstants.MAX_VEL / 1.35;
    double slowAcc = DriveConstants.MAX_ACCEL;

    Pose2d startPose = new Pose2d(-36, -65.25, Math.toRadians(270));

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);
        SampleMecanumDrive drive = initDriveOpMode();

        boolean[] driveVariables = initWithController(true);
//        boolean[] driveVariables = {true, false, true};
        if (driveVariables[2]) {
            parkY = -8;
        } else {
            parkY = -60;
        }
        robot.CLAW.setPosition(0);
        robot.LFS.setPosition(.97); // To swivel in more, increase this
        robot.RFS.setPosition(0.03);// To swivel in more, decrease this
        Pose2d backboard1 = new Pose2d(backboardX, -28.5, Math.toRadians(0));
        Pose2d backboard2 = new Pose2d(backboardX, -33.5, Math.toRadians(0));//35
        Pose2d backboard3 = new Pose2d(backboardX, -37.5, Math.toRadians(0));

        telemetry.addLine("Building Trajectories");
        telemetry.update();

        TrajectorySequence purple1 = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-47.65, -33.2), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(0, this::purpleIntake)
                .waitSeconds(0.1)
                .setTangent(270)
                .splineToConstantHeading(new Vector2d(-48.65, -33.2 - 12), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-59.5, -24, Math.toRadians(0)), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeWhite1())
                .UNSTABLE_addTemporalMarkerOffset(.6, () -> intakeTwoWhite2())
                .UNSTABLE_addTemporalMarkerOffset(.7, () -> {
                    robot.LFS.setPosition(.95); // To swivel in more, increase this
                    robot.RFS.setPosition(0.05);// To swivel in more, decrease this
                    lift(-1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> robot.CLAW.setPosition(0.5))
                .UNSTABLE_addTemporalMarkerOffset(1.3, () -> {
                    robot.LL.setPower(.2 * -1);
                    robot.RL.setPower(.2 * -1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.3, () -> intakeWhite3())
                .forward(1)
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(1.6, () -> {
                    robot.LL.setPower(0);
                    robot.RL.setPower(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(.05, () -> intakeWhite4())
                .strafeRight(2)
                .splineToConstantHeading(new Vector2d(-48, -60), Math.toRadians(0))
                .waitSeconds(0.0001)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeWhite1())
                .lineToConstantHeading(new Vector2d(6, -60))
                .splineToSplineHeading(new Pose2d(24, -48, Math.toRadians(30)), Math.toRadians(30))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeWhite4())
                .build();
        TrajectorySequence purple2 = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0,this::pushDown)
//                .strafeRight(4)
                .UNSTABLE_addTemporalMarkerOffset(.63, this::purpleIntake)
                .lineToConstantHeading(new Vector2d(-36, -33.5))
                .waitSeconds(0.1)
//                .splineToSplineHeading(new Pose2d(-36,-42,Math.toRadians(315)),Math.toRadians(315))
                .forward(6)
                .splineToSplineHeading(new Pose2d(-59.75, -24, Math.toRadians(0)), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeWhite1())
                .UNSTABLE_addTemporalMarkerOffset(.6, () -> intakeTwoWhite2())
                .UNSTABLE_addTemporalMarkerOffset(.7, () -> {
                    robot.LFS.setPosition(.95); // To swivel in more, increase this
                    robot.RFS.setPosition(0.05);// To swivel in more, decrease this
//                    lift(-1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.3, () -> intakeWhite3())
                .UNSTABLE_addTemporalMarkerOffset(2, () -> intakeWhite4())
//                .forward(1)
                .turn(Math.toRadians(20))
                .turn(Math.toRadians(-40))
                .turn(Math.toRadians(20))
//                .UNSTABLE_addTemporalMarkerOffset(1.6, () -> {
//                    robot.LL.setPower(0);
//                    robot.RL.setPower(0);
//                })
                .splineToConstantHeading(new Vector2d(-48, -60), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> robot.CLAW.setPosition(0.5))
                .waitSeconds(0.0001)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeWhite1())
                .splineTo(new Vector2d(6, -60),Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(24, -48, Math.toRadians(30)), Math.toRadians(30))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeWhite4())
                .build();
        // Start Intake
        TrajectorySequence purple3 = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-37.5, -40, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-33.5, -29), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, this::purpleIntake)
                .waitSeconds(0.1)
                .lineToLinearHeading(new Pose2d(-59.5, -24.1, Math.toRadians(0.1)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeWhite1())
                .UNSTABLE_addTemporalMarkerOffset(.2, () -> intakeTwoWhite2())
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> robot.CLAW.setPosition(0))
                .UNSTABLE_addTemporalMarkerOffset(.6, () -> robot.CLAW.setPosition(0.5))
                .UNSTABLE_addTemporalMarkerOffset(.6, () -> intakeWhite4())
                .forward(1)
                .turn(30)
                .turn(-60)
                .turn(30)
                .UNSTABLE_addTemporalMarkerOffset(1.6, () -> {
                    robot.LL.setPower(0);
                    robot.RL.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(-48, -60), Math.toRadians(0))
                .waitSeconds(0.0001)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeWhite1())
                .lineToConstantHeading(new Vector2d(6, -60))
                .splineToSplineHeading(new Pose2d(24, -48, Math.toRadians(30)), Math.toRadians(30))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeWhite4())
                .build();

        TrajectorySequence white1 = drive.trajectorySequenceBuilder(purple1.end())
                .back(5)
                .splineToSplineHeading(new Pose2d(10, -57, Math.toRadians(0)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-36, -56, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-58.5, -36, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.LFS.setPosition(.97); // To swivel in more, increase this
                    robot.RFS.setPosition(0.03);// To swivel in more, decrease this
                    lift(-1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intake1White1(3))
                .UNSTABLE_addTemporalMarkerOffset(.6, this::intakeTwoWhite2)
                .UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    robot.LFS.setPosition(.95); // To swivel in more, increase this
                    robot.RFS.setPosition(0.05);// To swivel in more, decrease this
                    lift(-1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> robot.CLAW.setPosition(0.5))
                .UNSTABLE_addTemporalMarkerOffset(1.6, () -> {
                    robot.LL.setPower(-.2);
                    robot.RL.setPower(-.2);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.1, this::intakeWhite3)
                .UNSTABLE_addTemporalMarkerOffset(2.5, this::intakeWhite4)
                .forward(1)
                .waitSeconds(1)
//                .UNSTABLE_addTemporalMarkerOffset(1.6, () -> {
//                    robot.LL.setPower(0);
//                    robot.RL.setPower(0);
//                })
                .splineToSplineHeading(new Pose2d(-36, -57.5, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, this::intakeWhite1)
                .splineToSplineHeading(new Pose2d(4, -54.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(15, -55, Math.toRadians(30)), Math.toRadians(30))
                .UNSTABLE_addTemporalMarkerOffset(0, this::intakeWhite4)
                .build();
        TrajectorySequence white2 = drive.trajectorySequenceBuilder(backboard2)
//                .back(5)
                .setTangent(180)
                .UNSTABLE_addTemporalMarkerOffset(0,this::pushDown)
                .splineToSplineHeading(new Pose2d(10, -57, Math.toRadians(0)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-36, -56, Math.toRadians(0)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-59.2, -35, Math.toRadians(0)), Math.toRadians(120))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.LFS.setPosition(.97); // To swivel in more, increase this
                    robot.RFS.setPosition(0.03);// To swivel in more, decrease this
                    lift(-1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intake1White1(2))
                .UNSTABLE_addTemporalMarkerOffset(.6, this::intakeTwoWhite2)
                .UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    robot.LFS.setPosition(.95); // To swivel in more, increase this
                    robot.RFS.setPosition(0.05);// To swivel in more, decrease this
//                    lift(-1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> robot.CLAW.setPosition(0.5))
                .UNSTABLE_addTemporalMarkerOffset(1.6, () -> {
                    robot.LL.setPower(-.2);
                    robot.RL.setPower(-.2);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.1, this::intakeWhite3)
                .UNSTABLE_addTemporalMarkerOffset(2.5, this::intakeWhite4)
                .turn(Math.toRadians(25))
                .turn(Math.toRadians(-50))
                .turn(Math.toRadians(25))
//                .UNSTABLE_addTemporalMarkerOffset(1.6, () -> {
//                    robot.LL.setPower(0);
//                    robot.RL.setPower(0);
//                })
                .splineToSplineHeading(new Pose2d(-36, -57.5, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, this::intakeWhite1)
                .splineToSplineHeading(new Pose2d(4, -54.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(15, -55, Math.toRadians(30)), Math.toRadians(30))
                .UNSTABLE_addTemporalMarkerOffset(0, this::intakeWhite4)
                .build();
        TrajectorySequence white3 = drive.trajectorySequenceBuilder(purple3.end())
                .back(5)
                .splineToSplineHeading(new Pose2d(10, -57, Math.toRadians(0)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-36, -56, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-53.5, -36, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.LFS.setPosition(.97); // To swivel in more, increase this
                    robot.RFS.setPosition(0.03);// To swivel in more, decrease this
                    lift(-1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intake1White1(3))
                .UNSTABLE_addTemporalMarkerOffset(.6, this::intakeTwoWhite2)
                .UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    robot.LFS.setPosition(.95); // To swivel in more, increase this
                    robot.RFS.setPosition(0.05);// To swivel in more, decrease this
                    lift(-1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> robot.CLAW.setPosition(0.5))
                .UNSTABLE_addTemporalMarkerOffset(1.6, () -> {
                    robot.LL.setPower(-.2);
                    robot.RL.setPower(-.2);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.1, this::intakeWhite3)
                .UNSTABLE_addTemporalMarkerOffset(2.5, this::intakeWhite4)
                .forward(1)
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(1.6, () -> {
                    robot.LL.setPower(0);
                    robot.RL.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(-36, -57.5), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, this::intakeWhite1)
                .splineToConstantHeading(new Vector2d(4, -54.5), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(15, -55, Math.toRadians(30)), Math.toRadians(30))
                .UNSTABLE_addTemporalMarkerOffset(0, this::intakeWhite4)
                .build();
//        TrajectorySequence park1 = drive.trajectorySequenceBuilder(white1.end())
//                .back(12)
//                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
//                    resetForTeleOp(liftHeight);
//                })
//                .lineToConstantHeading(new Vector2d(52, parkY))
//                .forward(10)
//                .build();
//        TrajectorySequence park2 = drive.trajectorySequenceBuilder(white2.end())
//                .back(12)
//                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
//                    resetForTeleOp(liftHeight);
//                })
//                .lineToConstantHeading(new Vector2d(52, parkY))
//                .forward(10)
//                .build();
//        TrajectorySequence park3 = drive.trajectorySequenceBuilder(white3.end())
//                .back(12)
//                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
//                    resetForTeleOp(liftHeight);
//                })
//                .lineToConstantHeading(new Vector2d(52, parkY))
//                .forward(10)
//                .build();

        telemetry.addLine("Starting Cameras...");
        telemetry.update();

        ColorPipeline pipeline = startCameras(false);
        while (!isStopRequested() && opModeInInit()) {
            sleep(1000);
            imageNum = pipeline.getImageNum();
            telemetry.addData("imageNum", imageNum);
            telemetry.update();
        }

        telemetry.addLine("Ready");
        telemetry.update();

//        imageNum = 3;

        waitForStart();
        closeColorPipelineCamera();

        drive.setPoseEstimate(startPose);

        TrajectorySequence purple = null;
        TrajectorySequence yellow = null;
        TrajectorySequence white = null;
        TrajectorySequence park = null;
        Pose2d backboardPose = null;
        Pose2d wbackboardPose = null;

        if (imageNum == 1) {
            purple = purple1;
            white = white1;
//            park = park1;
            backboardPose = new Pose2d(backboardX, -24.5, Math.toRadians(0));
            wbackboardPose = new Pose2d(backboardX, -36, Math.toRadians(0));
        } else if (imageNum == 2 || imageNum == 3 || imageNum == 4) {
            purple = purple2;
            white = white2;
//            park = park2;
            backboardPose = backboard2;
            wbackboardPose = backboard3;
        } else if (imageNum == 5) {
            purple = purple3;
            white = white3;
//            park = park3;
            backboardPose = backboard3;
            wbackboardPose = backboard2;
        }

//        waitForAprilTagCamera();
        resumeStreaming();

        telemetry.addLine("Trajectory Started.");
        telemetry.update();
        drive.followTrajectorySequence(purple);
        if (driveVariables[0] || driveVariables[1]) { 
            drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(relocalize(false))
                            .UNSTABLE_addTemporalMarkerOffset(0, () -> prepareScoring(liftHeight))
                            .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                                // Swivel out
                                robot.LFS.setPosition(0.60); // To swivel out more, decrease this
                                robot.RFS.setPosition(0.40); // To swivel out more, increase this
                            })
                            .lineToLinearHeading(backboardPose)
                            .UNSTABLE_addTemporalMarkerOffset(0, () -> scorePixelsOnBackboard(true))
                            .waitSeconds(.5)
                            .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                                resetForTeleOp1(liftHeight);
                            })
                            .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                                resetForTeleOp2(liftHeight);
                            })
                            .UNSTABLE_addTemporalMarkerOffset(1, () -> lift(-liftHeight))
                            .build());
        }
            if (driveVariables[1]) {
                // Follow White Once
                setTargetDropdownHeight(4);
                drive.followTrajectorySequence(white);
//                Pose2d correctPose = relocalize(false);
                drive.followTrajectorySequence(
                        drive.trajectorySequenceBuilder(relocalize(false))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> prepareScoring(liftHeight + 5))
                                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                                    // Swivel out
                                    robot.LFS.setPosition(0.60); // To swivel out more, decrease this
                                    robot.RFS.setPosition(0.40); // To swivel out more, increase this
                                })
                                .lineToLinearHeading(wbackboardPose)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> scorePixelsOnBackboard(false))
                                .waitSeconds(.5)
                                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                                    resetForTeleOp1(liftHeight);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                                    resetForTeleOp2(liftHeight);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(1, () -> lift(-liftHeight))
                                .build());
            }
//        drive.followTrajectorySequence(park);
    }
}

