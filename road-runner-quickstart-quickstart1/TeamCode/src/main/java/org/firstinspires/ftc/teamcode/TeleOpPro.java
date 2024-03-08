package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOpPro", group = "Drive")
public class TeleOpPro extends LinearOpMode {

    // Class Gamepad Documentation:
    // https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/Gamepad.html

    HardwarePushBot robot = new HardwarePushBot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        robot.FLD.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.FRD.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.BLD.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.BRD.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        robot.LL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.RL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        telemetry.setMsTransmissionInterval(5);

        waitForStart();
        double speed = 1.0;

        double lefty;
        double righty;
        double leftx;
        double rightx;

        boolean buttonL;
        boolean buttonR;

        double triggerR;
        double triggerL;
        boolean isSwiveledIn = true;

        double intakeSpeed;
        double intakeSpeedMultiplier = 1;

        double armPower;
        double armSpeed = 1.0;

        long nextDropdownTimestamp = 0;
        double dropdownPos = 0;
        double prevDropdownPos = 0;
        double pixelHeightOffset = 0.027; // Make sure 0.35 - pixelHeightOffset * 5 + dropROffset is not less than 0.
//        double dropLOffset = 0.04;
        double dropLOffset = 0.02;
        double dropROffset = 0;

        boolean dpadUp;
        boolean dpadDown;
        boolean dpadRight;
        boolean dpadLeft;

        boolean a;
        boolean b;
        boolean x;
        boolean y;

        long nextWristTimestamp = 0;
        long nextClawTimestamp = 0;
        boolean isClawOpen = true;
        boolean isWristVertical = true;

        long timestampTimeout = 300;

        // Initialize Servo Positions
        robot.CLAW.setPosition(0.1);
        robot.WRIST.setPosition(0.38);
        robot.LFS.setPosition(0.92); // To swivel in more, increase this
        robot.RFS.setPosition(0.08);// To swivel in more, decrease this
        // Set Dropdown to Level 5
        robot.DROPL.setPosition(0.35 + dropLOffset);
        robot.DROPR.setPosition(0.00 + dropROffset);

        while (opModeIsActive()) {
            // Gamepad 1
            lefty = -gamepad1.left_stick_y;
            righty = -gamepad1.right_stick_y;
            leftx = gamepad1.left_stick_x;
            rightx = gamepad1.right_stick_x;

            // Gamepad 2
            armPower = gamepad2.left_stick_y;
            intakeSpeed = gamepad2.right_stick_y;

            buttonL = gamepad2.left_bumper;
            buttonR = gamepad2.right_bumper;

            triggerL = gamepad2.left_trigger;
            triggerR = gamepad2.right_trigger;

            dpadUp = gamepad2.dpad_up;
            dpadLeft = gamepad2.dpad_left;
            dpadRight = gamepad2.dpad_right;
            dpadDown = gamepad2.dpad_down;

            y = gamepad2.y;
            x = gamepad2.x;
            a = gamepad2.a;
            b = gamepad2.b;

            // Gamepad 1 Code
            if (gamepad1.a) {
                speed = 1.0;
            } else if (gamepad1.b) {
                speed = 0.75;
            } else if (gamepad1.x) {
                speed = 0.50;
            } else if (gamepad1.y) {
                speed = 0.30;
            }

            if (gamepad1.right_bumper) {
                robot.AP.setPosition(0);
            }

            if (gamepad2.left_bumper) {
                robot.AP.setPosition(1);
            }

            // Gamepad 2 Code

            // TODO: Fix CLAW's 0 position since it doesn't hold pixels
            if (buttonR && gamepad2.timestamp > nextClawTimestamp) {
                // Control CLAW
                nextClawTimestamp = gamepad2.timestamp + timestampTimeout;

                if (isClawOpen) {
                    // Set CLAW to close position
                    robot.CLAW.setPosition(0.1);
                } else {
                    // Set CLAW to open position
                    robot.CLAW.setPosition(0.5);
                }
                isClawOpen = !isClawOpen;
            }

            if (!isSwiveledIn && buttonL && gamepad2.timestamp > nextWristTimestamp) {
                // Rotate WRIST
                nextWristTimestamp = gamepad2.timestamp + timestampTimeout;

                if (isWristVertical) {
                    // Set WRIST to horizontal position
                    robot.WRIST.setPosition(0.73);
                } else {
                    // Set WRIST to vertical position
                    robot.WRIST.setPosition(0.38);
                }
                isWristVertical = !isWristVertical;
            }

            if (triggerL > 0) {
                // Swivel in
                robot.WRIST.setPosition(0.38); // Set wrist to vertical
                robot.LFS.setPosition(0.92); // To swivel in more, increase this
                robot.RFS.setPosition(0.08);// To swivel in more, decrease this
                isSwiveledIn = true;
            }

            if (triggerR > 0) {
                // Swivel out
                robot.LFS.setPosition(0.50); // To swivel out more, decrease this
                robot.RFS.setPosition(0.50); // To swivel out more, increase this
                isSwiveledIn = false;
            }

            if (dpadUp && dropdownPos < 5 && gamepad2.timestamp > nextDropdownTimestamp) {
                nextDropdownTimestamp = gamepad2.timestamp + timestampTimeout;
                dropdownPos++;
            } 
            if (dpadDown && dropdownPos > 0 && gamepad2.timestamp > nextDropdownTimestamp) {
                nextDropdownTimestamp = gamepad2.timestamp + timestampTimeout;
                dropdownPos--;
            }
            
            if (dpadLeft) {
                dropdownPos = 0;
            }
            if (dpadRight) {
                dropdownPos = 5;
            }

            if (x) {
                intakeSpeedMultiplier = 1.0;
            }

            if (y) {
                intakeSpeedMultiplier = 0.50;
            }

            if (b) {
                // Set CLAW to open position
                robot.CLAW.setPosition(0.5);
                // Set WRIST to vertical position
                robot.WRIST.setPosition(0.38);
                // Swivel in
                robot.LFS.setPosition(0.95); // To swivel in more, increase this
                robot.RFS.setPosition(0.05);// To swivel in more, decrease this
            }

            // Gamepad 1
            double FLDp = lefty + leftx;
            double FRDp = righty - rightx;
            double BLDp = lefty - leftx;
            double BRDp = righty + rightx;

            robot.FLD.setPower(speed * FLDp);
            robot.FRD.setPower(speed * FRDp);
            robot.BLD.setPower(speed * BLDp);
            robot.BRD.setPower(speed * BRDp);

            // Gamepad 2
            if (dropdownPos != prevDropdownPos) {
                if (dropdownPos == 0) {
                    robot.DROPL.setPosition(0.00 + pixelHeightOffset * 1 + dropLOffset);
                    robot.DROPR.setPosition(0.35 - pixelHeightOffset * 1 + dropROffset);
                } else if (dropdownPos == 1) {
                    robot.DROPL.setPosition(0.00 + pixelHeightOffset * 2 + dropLOffset);
                    robot.DROPR.setPosition(0.35 - pixelHeightOffset * 2 + dropROffset);
                } else if (dropdownPos == 2) {
                    robot.DROPL.setPosition(0.00 + pixelHeightOffset * 3 + dropLOffset);
                    robot.DROPR.setPosition(0.35 - pixelHeightOffset * 3 + dropROffset);
                } else if (dropdownPos == 3) {
                    robot.DROPL.setPosition(0.00 + pixelHeightOffset * 4 + dropLOffset);
                    robot.DROPR.setPosition(0.35 - pixelHeightOffset * 4 + dropROffset);
                } else if (dropdownPos == 4) {
                    robot.DROPL.setPosition(0.00 + pixelHeightOffset * 5 + dropLOffset);
                    robot.DROPR.setPosition(0.35 - pixelHeightOffset * 5 + dropROffset);
                } else if (dropdownPos == 5) {
                    robot.DROPL.setPosition(0.35 + dropLOffset);
                    robot.DROPR.setPosition(0.00 + dropROffset);
                }
                prevDropdownPos = dropdownPos;
            }
            robot.IN.setPower(intakeSpeed * intakeSpeedMultiplier);
            robot.LL.setPower(armSpeed * armPower);
            robot.RL.setPower(armSpeed * armPower);

            telemetry.update();
        }
    }
}
