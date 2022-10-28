package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Mecanum drive: Robot Centric", group="Linear Opmode")
public class MecanumTeleOp extends LinearOpMode {

    private final double ZERO_POWER = 0.0;

    // Declare OpMode members
    private DcMotor motorFrontLeft = null;
    private DcMotor motorBackLeft = null;
    private DcMotor motorFrontRight = null;
    private DcMotor motorBackRight = null;

    private DcMotor linearSlide = null;
    private Servo grabber = null;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        grabber = hardwareMap.servo.get("grabberServo");

        // Additional functionality
        linearSlide = hardwareMap.dcMotor.get("linearSlide");
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            drive();
            // use either one of the two
//            slideBumpers();
            slideTrigger();
            slideEncoderTarget();
            grabber();
            telemetry.update();
        }
    }

    private void drive() {
        double speedFactor = gamepad1.left_trigger * 1;
        telemetry.addData("left_trigger (speedFactor): ", gamepad1.left_trigger);

        double y = gamepad1.left_stick_y; // Remember, this is reversed!
        double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = -gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        denominator = denominator * (1 + speedFactor);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        motorFrontLeft.setPower(frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackRight.setPower(backRightPower);

        telemetry.addData("Front Left Motor: ", frontLeftPower);
        telemetry.addData("Front Right Motor: ", frontRightPower);
        telemetry.addData("Back Left Motor: ", backLeftPower);
        telemetry.addData("Back Right Motor: ", backRightPower);
    }

    // Using left and right bumper to move the slider at a fixed power
    private void slideBumpers() {
        double power = 0.5;
        if (gamepad2.right_bumper) {
            linearSlide.setPower(power);
        } else if (gamepad2.left_bumper) {
            linearSlide.setPower(-power);
        } else {
            linearSlide.setPower(ZERO_POWER);
        }
    }

    // Using left and right trigger to move the slider based on power;
    private void slideTrigger() {
        double constant = 1.0;
        double forwardPower = gamepad2.right_trigger * constant;
        double reversePower = gamepad2.left_trigger * constant;
        // Set the mode to run without encoders if manual control is detected
        if (forwardPower != 0.0 || reversePower != 0.0) {
            linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        double powerLimiter = 0.5; // limits the power the motors can run at so from -0.5 to 0.5
        double power = (forwardPower != ZERO_POWER) ? forwardPower : -1 * reversePower;
//        if (forwardPower != ZERO_POWER) {
//            power = forwardPower;
//        } else {
//            power = -1 * reversePower;
//        }
        // TODO: set minimum and maximum power ranges if we don't want the motors to go too fast
        // default is -1.0 -> 1.0
        power = Range.clip(power, -1 * powerLimiter, powerLimiter);
        linearSlide.setPower(power);
        telemetry.addData("Slide Power: ", linearSlide.getPower());
        telemetry.addData("Encoder Distance: ", linearSlide.getCurrentPosition());
    }

    private void slideEncoderTarget() {
        // If gamepad 2 buttons a,x,y are pressed, we will use encoders to reach target
        if (gamepad2.a || gamepad2.x || gamepad2.y) {
            // Stop and reset the encoders
            linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Encoder settings
            int shortPole = 1900, medPole = 4000, highPole = 5000;
            if (gamepad2.a) linearSlide.setTargetPosition(shortPole);
            else if (gamepad2.x) linearSlide.setTargetPosition(medPole);
            else if (gamepad2.y) linearSlide.setTargetPosition(highPole);

            // If a target position is set, run to that position
            if (linearSlide.getTargetPosition() != 0) {
                linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // Motor will run at the designated power until it reaches the position
                double power = 0.25;
                linearSlide.setPower(power);

                // Wait until the motors stop.
                while (linearSlide.isBusy()) {
                    telemetry.addData("Linear Slide Distance: ", linearSlide.getCurrentPosition());
                    telemetry.update();
                    idle();
                }

                // Set power back to 0.0 since position is reached.
                // TODO: need to find the power to counteract gravitational pull
                linearSlide.setPower(ZERO_POWER);
            }
        }
    }

    private void grabber() {
        if (gamepad1.a) {
            grabber.setPosition(0);
        } else if (gamepad1.b) {
            grabber.setPosition(1);
        }
        telemetry.addData("Grabber Position: ", grabber.getPosition());
        telemetry.update();
    }
}