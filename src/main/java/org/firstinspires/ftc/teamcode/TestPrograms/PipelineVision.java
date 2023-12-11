package org.firstinspires.ftc.teamcode.TestPrograms;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class PipelineVision extends OpenCvPipeline{

        boolean viewportPaused;
        public int actualred;
        public int position;

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        @Override
        public Mat processFrame(Mat input) {
            Mat HSV = new Mat();
            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

            Mat cropped1 = HSV.submat(new Rect(0, 0, 160, 120));
            Mat cropped2 = HSV.submat(new Rect(160, 120, 160, 120));


            List<Mat> cropped1split = new ArrayList<>();
            Core.split(cropped1, cropped1split);

            List<Mat> cropped2split = new ArrayList<>();
            Core.split(cropped2, cropped2split);

            int red = 0, green = 0, blue = 0;
            int red2 = 0, blue2 = 0;


            for (int i = 0; i < cropped1split.get(0).rows(); i++) {
                for (int j = 0; j < cropped1split.get(0).cols(); j++) {
                    if (cropped1split.get(2).get(i, j)[0] > 50 && cropped1split.get(1).get(i, j)[0] > 45) {
                        if (cropped1split.get(0).get(i, j)[0] > 110 && cropped1split.get(0).get(i, j)[0] < 125) {
                            blue++;
                        }
                    }
                    if (cropped1split.get(2).get(i, j)[0] > 50 && cropped1split.get(1).get(i, j)[0] > 45) {
                        if (cropped1split.get(0).get(i, j)[0] > 0 && cropped1split.get(0).get(i, j)[0] < 6) {
                            red++;
                        }
                        if (cropped1split.get(0).get(i, j)[0] > 173 && cropped1split.get(0).get(i, j)[0] < 179) {
                            red++;
                        }
                    }
                }

            }

            for (int i = 0; i < cropped2split.get(0).rows(); i++) {
                for (int j = 0; j < cropped2split.get(0).cols(); j++) {
                    if (cropped2split.get(2).get(i, j)[0] > 50 && cropped2split.get(1).get(i, j)[0] > 45) {
                        if (cropped2split.get(0).get(i, j)[0] > 110 && cropped2split.get(0).get(i, j)[0] < 125) {
                            blue2++;
                        }
                    }
                    if (cropped2split.get(2).get(i, j)[0] > 50 && cropped2split.get(1).get(i, j)[0] > 45) {
                        if (cropped2split.get(0).get(i, j)[0] > 0 && cropped2split.get(0).get(i, j)[0] < 6) {
                            red2++;
                        }
                        if (cropped2split.get(0).get(i, j)[0] > 173 && cropped2split.get(0).get(i, j)[0] < 179) {
                            red2++;
                        }
                    }
                }

            }

            if (red2 > 3000) {
                position = 1; // Cropped 2
            } else if (red > 3000) {
                position = 2; //position in cropped
            } else {
                position = 3; //position is last position not in camera
            }

            actualred = red;

            int g = 0, b = 0, r = 0;
            int max = 0;
            if (red > blue && red > green)
                max = red;
            else if (green > red && green > blue)
                max = green;
            else if (blue > red && blue > green)
                max = blue;

            if (max > 5000) {

            }
            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols() / 4,
                            input.rows() / 4),
                    new Point(
                            input.cols() * (3f / 4f),
                            input.rows() * (3f / 4f)),
                    new Scalar(255, g, b), 4);

            return input;
        }
    }
