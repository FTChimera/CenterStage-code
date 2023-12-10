package org.firstinspires.ftc.teamcode.programs;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@TeleOp
public class Canyoony extends LinearOpMode {
    SampleMecanumDrive drive;
    boolean locked = false;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("motorBackRight");
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());
        waitForStart();
        while (opModeIsActive()){
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


                lockToPosition(new Pose2d());

            telemetry.addData("Locked?",locked);
            telemetry.update();
            drive.update();
        }

    }
    public void lockToPosition(Pose2d targetPose){
        Pose2d currentPose = drive.getPoseEstimate();
        Pose2d diff = targetPose.minus(currentPose);
        Vector2d xy = diff.vec().rotated(-currentPose.getHeading());
        double heading = Angle.normDelta(targetPose.getHeading()) - Angle.normDelta(currentPose.getHeading());
        drive.setWeightedDrivePower(new Pose2d(xy,heading));
    }
}