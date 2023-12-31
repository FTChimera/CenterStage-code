package org.firstinspires.ftc.teamcode.programs;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
public class EasyOpenCvPipeline extends OpenCvPipeline {
    boolean viewportPaused;
    Mat yCrCb = new Mat();
    Mat leftCrop;
    Mat rightCrop;
    public double leftValue, rightValue;
    Rect rectLeft;
    Rect rectRight;
    Mat output = new Mat();
    Scalar rectColor;
    OpenCvWebcam webcam;
    double threshHold;
    public EasyOpenCvPipeline(OpenCvWebcam webcam){
        this.webcam = webcam;
    }
    public void setAlliance(int alliance){
        if (alliance == 1){
            rectColor = new Scalar(79, 174, 205);
            rectLeft = new Rect(180,185,20,20);// rectangles need to be swapped for blue side
            rectRight = new Rect(10,175, 20,20);
            threshHold = 10;
        }
        else {
            rectColor = new Scalar(255.0, 0.0, 0.0);
            rectLeft = new Rect(10, 183, 20, 20);
            rectRight = new Rect(180, 187, 20, 20);
            threshHold = 20;
        }
    }
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input,yCrCb,Imgproc.COLOR_RGB2YCrCb);
        input.copyTo(output);
        Imgproc.rectangle(output,rectLeft,rectColor,2);
        Imgproc.rectangle(output,rectRight,rectColor,2);
        leftCrop = yCrCb.submat(rectLeft);
        rightCrop = yCrCb.submat(rectRight);
        Core.extractChannel(leftCrop, leftCrop, 1);
        Core.extractChannel(rightCrop,rightCrop,1);
        leftValue = Core.mean(leftCrop).val[0];
        rightValue = Core.mean(rightCrop).val[0];
        // to prevent memory leaks
        leftCrop.release();
        rightCrop.release();
        return output;
    }
    public int getLocation(){
        if (Math.abs(rightValue-leftValue) < threshHold){// checks for left
            return 1;
        }
        else if ( rightValue > leftValue ){// checks for right
            return 3;
        }
        else return 2;
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
    }
}