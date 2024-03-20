//package org.firstinspires.ftc.teamcode.tests;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//import org.firstinspires.ftc.teamcode.resources.DriveOpMode;
//
//@TeleOp
//public class TestColorPipeline extends DriveOpMode {
//    @Override
//    public void runOpMode() {
//        telemetry.setMsTransmissionInterval(50);
//        SampleMecanumDrive drive = initDriveOpMode();
//
//        boolean blue = true;
//        while (!gamepad1.right_bumper) {
//          if (gamepad1.x) {
//            blue = false;
//          }
//          if (gamepad1.y) {
//            blue = true;
//          }
//          sleep(10);
//        }
//
//        ColorPipeline pipeline = startCameras(blue);
//        while (!isStopRequested() && opModeInInit()) {
//            sleep(1000);
//            imageNum = pipeline.getImageNum();
//            telemetry.addData("imageNum", imageNum);
//            telemetry.update();
//        }
//    }
//}
