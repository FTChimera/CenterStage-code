package org.firstinspires.ftc.teamcode.programs;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

@Config
@Autonomous
public class BlueStacks extends LinearOpMode {
    OpenCvWebcam webcam;
    EasyOpenCvPipeline pipeline;
    double headingError;
    double rangeError;
    ElapsedTime timer = new ElapsedTime();
    int DESIRED_TAG_ID = 1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag; // Used to hold the data for a detected AprilTag
    boolean tagFound;
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
        Servo tiltServo = hardwareMap.servo.get("tiltServo");

        drive.setPoseEstimate(new Pose2d(14, -58, Math.toRadians(90)));
        TrajectorySequence elementInLeft = drive.trajectorySequenceBuilder(new Pose2d(14, -58, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(13, -30, Math.toRadians(180)))
                .forward(5)
                .back(3)
                .addTemporalMarker(() -> rollerIntake.setPower(-1))
                .addTemporalMarker(() -> intakeMotor.setPower(-0.1))
                .waitSeconds(1.25)
                .addTemporalMarker(() -> rollerIntake.setPower(0))
                .addTemporalMarker(() -> intakeMotor.setPower(0))
                .strafeTo(new Vector2d(40.5, -30))
                .build();
        TrajectorySequence elementInMiddle = drive.trajectorySequenceBuilder(new Pose2d(14, -58, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(12, -28))
                .back(4)
                .addTemporalMarker(() -> rollerIntake.setPower(-1))
                .addTemporalMarker(() -> intakeMotor.setPower(-0.1))
                .waitSeconds(1.25)
                .addTemporalMarker(() -> rollerIntake.setPower(0))
                .addTemporalMarker(() -> intakeMotor.setPower(0))
                .build();
        TrajectorySequence elementInRight = drive.trajectorySequenceBuilder(new Pose2d(14, -58, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(21.5, -40))
                .forward(5)
                .back(3)
                .addTemporalMarker(() -> rollerIntake.setPower(-1))
                .addTemporalMarker(() -> intakeMotor.setPower(-0.1))
                .waitSeconds(1.25)
                .addTemporalMarker(() -> rollerIntake.setPower(0))
                .addTemporalMarker(() -> intakeMotor.setPower(0))
                .build();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new EasyOpenCvPipeline(webcam);
        pipeline.setAlliance(1);
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                flipDownServo.setPosition(-1);
                droneLauncher.setPosition(-1);
                grip1.setPosition(1);
                grip2.setPosition(-0.7);
                outtakeLeft.setPosition(1);
                outtakeRight.setPosition(-1);
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
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .addProcessor(aprilTag)
                .build();

    }
    public void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                waitSeconds(2);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                waitSeconds(2);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            waitSeconds(2);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            waitSeconds(2);
        }
    }
    public void waitSeconds(double seconds){
        timer.reset();
        while (timer.seconds() < seconds){

        }
    }

}

