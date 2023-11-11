package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "TeleOp", group = "TeleOps")
public class NewTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("motorBackRight");
        DcMotor rightSlide = hardwareMap.dcMotor.get("RightSlideMotor");
        DcMotor leftSlide = hardwareMap.dcMotor.get("LeftSlideMotor");
        CRServo intake1 = hardwareMap.crservo.get("intake1");
        CRServo intake2 = hardwareMap.crservo.get("intake2");
        CRServo flipDownIntake = hardwareMap.crservo.get("flipDownIntake");
        Servo outtakeLeft = hardwareMap.servo.get("outtakeLeft");
        Servo outtakeRight = hardwareMap.servo.get("outtakeRight");
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            double extraPower = 1.5;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower * extraPower);

            if (gamepad2.dpad_up && leftSlide.getCurrentPosition()<=1300) {
                leftSlide.setPower(1);
                rightSlide.setPower(1);
            } else if (gamepad2.dpad_down && leftSlide.getCurrentPosition()>=0) {
                leftSlide.setPower(-0.5);
                rightSlide.setPower(-0.5);
            }
            else {
                leftSlide.setPower(0.1);
                rightSlide.setPower(0.1);
            }

            while (gamepad1.left_trigger > 0.5){ // outtake
                intake1.setPower(1);
                intake2.setPower(1);
                flipDownIntake.setPower(-1);
            }
            intake1.setPower(0);
            intake2.setPower(0);
            flipDownIntake.setPower(0);

            while (gamepad1.right_trigger > 0.5){ // intake
                intake1.setPower(-1);
                intake2.setPower(-1);
                flipDownIntake.setPower(1);
            }
            intake1.setPower(0);
            intake2.setPower(0);
            flipDownIntake.setPower(0);

            if (gamepad2.a){
                outtakeLeft.setPosition(1);
                outtakeRight.setPosition(-1);
            }
            if (gamepad2.b){
                outtakeLeft.setPosition(-1);
                outtakeRight.setPosition(1);
            }

            telemetry.addData("Left", leftSlide.getCurrentPosition());
            telemetry.addData("Right", rightSlide.getCurrentPosition());
            telemetry.update();

        }
    }
}

