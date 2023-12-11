package org.firstinspires.ftc.teamcode.TestPrograms;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
@Autonomous
public class RRTest extends LinearOpMode {
    SampleMecanumDrive drive;
    OpenCvWebcam webcamLeft, webcamMiddle, webcamRight;
    double turn = 0, turn2 = 0;
    EasyOpenCvPipeline pipelineLeft, pipelineMiddle, pipelineRight;
    CRServo intake1 , intake2;
    public void startWebcam(OpenCvWebcam webcam, EasyOpenCvPipeline pipeline){

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);






        pipeline = new EasyOpenCvPipeline(webcam);
        webcam.setPipeline(pipeline);




        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        OpenCvWebcam finalWebcam = webcam;
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {


                finalWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }


            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }


    @Override
    public void runOpMode() throws InterruptedException {
         intake1 = hardwareMap.crservo.get("intake1");
         intake2 = hardwareMap.crservo.get("intake2");
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());
        TrajectorySequence ts = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(45)
                .addDisplacementMarker(() -> {
                    startWebcam(webcamLeft, pipelineLeft);
                    startWebcam(webcamMiddle, pipelineMiddle);
                    startWebcam(webcamRight, pipelineRight);
                    if (pipelineLeft.value < pipelineMiddle.value && pipelineLeft.value < pipelineRight.value) {
                        turn = -90;
                        turn2 = Math.toRadians(180) + 1e-6;
                    }
                    else if (pipelineRight.value < pipelineMiddle.value && pipelineRight.value < pipelineLeft.value){
                        turn = 90;
                    }
                })


                .turn(Math.toRadians(turn))
                .forward(8)
                .addDisplacementMarker(() -> {
                    intake1.setPower(-1);
                    intake2.setPower(-1);
                    sleep(100);
                    intake1.setPower(0);
                    intake2.setPower(0);

                })
                .turn(turn2)
                .build();
        waitForStart();
        if (opModeIsActive()) {


            drive.followTrajectorySequence(ts);
        }


        return ;
    }
}
