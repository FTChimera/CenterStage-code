package org.firstinspires.ftc.teamcode.programs;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous
public class RedStacks extends LinearOpMode {
    OpenCvWebcam webcam;
    EasyOpenCvPipeline pipeline;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

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

        drive.setPoseEstimate(new Pose2d(-40, -58, Math.toRadians(90)));
        TrajectorySequence elementInLeft = drive.trajectorySequenceBuilder(new Pose2d(-40, -58, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(-35.5,-10, Math.toRadians(270)))
                .strafeTo(new Vector2d(-46.5,-20))
                .forward(5)
                .back(3)
                .waitSeconds(3)
                .setReversed(true)
                .splineTo(new Vector2d(-25,-10), Math.toRadians(0))
                .setReversed(false)
                .waitSeconds(10)
                .back(70)
                .strafeTo(new Vector2d(45, -28.75))
                .waitSeconds(1)
                //Deposit Pixel
                .waitSeconds(0.5)
                .forward(2)
                .strafeTo(new Vector2d(45, -10))
                .back(15)
                .build();
        TrajectorySequence elementInMiddle = drive.trajectorySequenceBuilder(new Pose2d(-40, -58, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(-35.5,-10, Math.toRadians(270)))
                .strafeTo(new Vector2d(-35.5,-13))
                .forward(5)
                .back(3)
                .waitSeconds(3)
                .setReversed(true)
                .splineTo(new Vector2d(-25,-10), Math.toRadians(0))
                .setReversed(false)
                .waitSeconds(10)
                .back(70)
                .strafeTo(new Vector2d(45, -28.75))
                .waitSeconds(1)
                //Deposit Pixel
                .waitSeconds(0.5)
                .forward(2)
                .strafeTo(new Vector2d(45, -34.5))
                .back(15)
                .build();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new EasyOpenCvPipeline(webcam);
        pipeline.setAlliance(2);
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
            }
        });
        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();



        if (opModeIsActive()) {
            while (pipeline.leftValue == 0 && pipeline.rightValue == 0){ // waits for webcam to start sensing

            }
            int position = pipeline.getLocation();
            if (position == 2) {
                drive.followTrajectorySequence(elementInMiddle);
                telemetry.addLine("Position: Middle");
            } else if (position == 3) {
                //drive.followTrajectorySequence(elementInRight);
                telemetry.addLine("Position: Right");
            }
            else {
                drive.followTrajectorySequence(elementInLeft);
                telemetry.addLine("Position: Left");
            }
            telemetry.update();
            webcam.stopStreaming();
        }
    }
}
