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
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous
public class RedBackboard extends LinearOpMode {
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
        Servo droneLauncher = hardwareMap.servo.get("droneLauncher");

        int slowerVelocity = 30;
        drive.setPoseEstimate(new Pose2d(14, -58, Math.toRadians(90)));
        TrajectorySequence elementInLeft = drive.trajectorySequenceBuilder(new Pose2d(14, -58, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(13, -32, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .forward(5,
                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .back(3,
                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(3)
                .strafeTo(new Vector2d(45, -28.75),
                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(8)
                //Deposit Pixel
                .forward(2,
                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .strafeTo(new Vector2d(45, -60),
                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .back(15,
                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

//                .addSpatialMarker(new Vector2d(13, -30), () -> {
//                    rollerIntake.setPower(-1);
//                })
//                .addSpatialMarker(new Vector2d(45, -28.75), () -> {
//                    rollerIntake.setPower(0);
//                })
//                .addSpatialMarker(new Vector2d(24, -31), () -> {
//                    leftSlide.setPower(1);
//                    rightSlide.setPower(1);
//                    outtakeLeft.setPosition(-1);
//                    outtakeRight.setPosition(1);
//                })
//                .addSpatialMarker(new Vector2d(30, -30.6), () -> {
//                    leftSlide.setPower(0);
//                    rightSlide.setPower(0);
//                })
//                .addSpatialMarker(new Vector2d(45, -28.75), () -> {
//                    grip1.setPosition(-0.5);
//                    grip1.setPosition(0.5);
//                })
//                .addSpatialMarker(new Vector2d(45, -40), () -> {
//                    outtakeLeft.setPosition(1);
//                    outtakeRight.setPosition(-1);
//                    leftSlide.setPower(-0.5);
//                    rightSlide.setPower(-0.5);
//                })
//                .addSpatialMarker(new Vector2d(45, -60), () -> {
//                    leftSlide.setPower(0);
//                    rightSlide.setPower(0);
//                })

                .build();
        TrajectorySequence elementInMiddle = drive.trajectorySequenceBuilder(new Pose2d(14, -58, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(12, -31))
                .back(2)
                .waitSeconds(3)
                .setReversed(true)
                .splineTo(new Vector2d(45, -34.5), Math.toRadians(0))
                .setReversed(false)
                .waitSeconds(1)
                //Deposit Pixel
                .forward(2)
                .strafeTo(new Vector2d(45, -60))
                .back(15)
                .build();
        TrajectorySequence elementInRight = drive.trajectorySequenceBuilder(new Pose2d(14, -58, Math.toRadians(90)))
                .strafeTo(new Vector2d(23,-40))
                .forward(5)
                .back(3)
                .waitSeconds(3)
                .setReversed(true)
                .splineTo(new Vector2d(45, -40.75), Math.toRadians(0))
                .setReversed(false)
                .waitSeconds(1)
                //Deposit Pixel
                .strafeTo(new Vector2d(45, -60))
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
            flipDownServo.setPosition(-1);
            droneLauncher.setPosition(-1);
            grip1.setPosition(1);
            grip2.setPosition(-0.7);
            outtakeLeft.setPosition(1);
            outtakeRight.setPosition(-1);
            }
            int position = pipeline.getLocation();
            if (position == 2) {
               drive.followTrajectorySequence(elementInMiddle);
               telemetry.addLine("Position: Middle");
            } else if (position == 3) {
               drive.followTrajectorySequence(elementInRight);
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
