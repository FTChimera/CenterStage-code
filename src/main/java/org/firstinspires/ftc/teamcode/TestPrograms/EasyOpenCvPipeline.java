package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class EasyOpenCvPipeline extends OpenCvPipeline {


    boolean viewportPaused;
    Mat hsv = new Mat();
    Mat leftCrop;

    int coi;


   public double value;




    Mat output = new Mat();
    Scalar rectColor = new Scalar(255.0,0.0,0.0);
    OpenCvWebcam webcam;


    public EasyOpenCvPipeline(OpenCvWebcam webcam){
        this.webcam = webcam;
    }

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input,hsv,Imgproc.COLOR_RGB2HSV);


        Rect rect = new Rect(1,1, 320,240);

        // (0,0) is top left corner of webcam
        // if using different webcam sizes, width = (CameraWidth/3)-1
        // y is half the height of webcam for each rectangle




        // CameraWidth is the width defined in webcam.startStreaming()
        // Camera Height is the height defined in webcam.startStreaming()
        // the difference between each x coordinate should be width




        // puts the rectangles onto the stream


        input.copyTo(output);
        Imgproc.rectangle(output,rect,rectColor,2);



        leftCrop = hsv.submat(rect);

// change coi to 1 for blue, 0 for red


        coi = 0;


        Core.extractChannel(leftCrop, leftCrop, coi);



        Scalar leftAvg = Core.mean(leftCrop);
        value = leftAvg.val[0];





        // to prevent memory leaks
        leftCrop.release();




        return output;
    }




    @Override
    public void onViewportTapped(){
        viewportPaused = !viewportPaused;


        if(viewportPaused)
        {
            webcam.pauseViewport();
        }
        else


            webcam.resumeViewport();
    }}


