// Package (Location of Code)
package org.firstinspires.ftc.teamcode.programs;

// Import Statements
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "SehujOp", group = "TeleOps")// Name and Group
public class SehujOp extends LinearOpMode {// Start of Code
    SampleMecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());

        // Initialize Hardware
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("motorBackRight");
        DcMotor rightSlide = hardwareMap.dcMotor.get("RightSlideMotor");
        DcMotor leftSlide = hardwareMap.dcMotor.get("LeftSlideMotor");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        Servo outtakeLeft = hardwareMap.servo.get("outtakeLeft");
        Servo outtakeRight = hardwareMap.servo.get("outtakeRight");
        CRServo rollerIntake = hardwareMap.crservo.get("rollerIntake");
        Servo flipDownServo = hardwareMap.servo.get("flipDownServo");
        CRServo flipDownIntake = hardwareMap.crservo.get("flipDownIntake");
        Servo grip1 = hardwareMap.servo.get("grip1");
        Servo grip2 = hardwareMap.servo.get("grip2");
        Servo droneLauncher = hardwareMap.servo.get("droneLauncher");
        Servo tiltServo = hardwareMap.servo.get("tiltServo");

        // Set Modes For Certain Motors
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reverse Motor Directions
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();// Wait for Play to Be Clicked

        if (isStopRequested()) return;// If Stop Is Clicked At Any Time, STOP

        while (opModeIsActive()) {// When Program is Running
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1; // Correct For Imperfect Strafing
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);// Math Stuff
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            // Set Wheel Power
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            if (gamepad2.dpad_up && leftSlide.getCurrentPosition()<=2050) {
                leftSlide.setPower(1);
                rightSlide.setPower(1);
            } else if (gamepad2.dpad_down && leftSlide.getCurrentPosition()>=0) {
                leftSlide.setPower(-0.5);
                rightSlide.setPower(-0.5);
            }
            else {
                leftSlide.setPower(0);
                rightSlide.setPower(0);
            }

            if (gamepad1.left_trigger > 0.5) {
                flipDownIntake.setPower(1);
                intakeMotor.setPower(1);
                rollerIntake.setPower(1);
            }

            if (gamepad1.left_bumper) {
                intakeMotor.setPower(0);
                flipDownIntake.setPower(0);
                rollerIntake.setPower(0);
            }

            if (gamepad1.right_trigger > 0.5) {
                intakeMotor.setPower(-1);
                rollerIntake.setPower(-1);
            }

            if (gamepad1.right_bumper) {
                intakeMotor.setPower(0);
                flipDownIntake.setPower(0);
                rollerIntake.setPower(0);
            }

            if (gamepad2.a){
                flipDownServo.setPosition(-1);
            }

            if (gamepad2.b){
                flipDownServo.setPosition(1);
            }

            if (gamepad2.y){// DOWN
                outtakeLeft.setPosition(1);
                outtakeRight.setPosition(-1);
                tiltServo.setPosition(-1);
            }
            if (gamepad2.x){ //UP
                outtakeLeft.setPosition(-1);
                outtakeRight.setPosition(1);
                tiltServo.setPosition(1);
            }

            if (gamepad1.x){
                grip1.setPosition(1);
                grip2.setPosition(-0.7);
            }

            if (gamepad1.a){
                grip1.setPosition(-0.5);
            }

            if (gamepad1.b){
                grip2.setPosition(0.5);
            }

            if (gamepad2.right_stick_button && gamepad1.right_stick_button){
                droneLauncher.setPosition(1);
            }

            if (gamepad1.left_stick_button){
                droneLauncher.setPosition(-1);
            }

            drive.update();// Update Telemetry
            Pose2d currentPose = drive.getPoseEstimate();
            telemetry.addData("X", currentPose.getX());
            telemetry.addData("Y", currentPose.getY());
            telemetry.addData("Heading", currentPose.getHeading());
            telemetry.addData("Left", leftSlide.getCurrentPosition());
            telemetry.addData("Right", rightSlide.getCurrentPosition());
            telemetry.update();
        }
    }
}