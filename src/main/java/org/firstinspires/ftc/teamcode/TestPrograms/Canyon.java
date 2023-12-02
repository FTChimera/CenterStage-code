package org.firstinspires.ftc.teamcode.TestPrograms;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
//@Disabled
public class Canyon extends LinearOpMode {

    SampleMecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            lockToPosition(new Pose2d(0,0,0));
        }

        return ;
    }
    public void lockToPosition(Pose2d targetPose){
        Pose2d currentPose = drive.getPoseEstimate();
        Pose2d diff = targetPose.minus(currentPose);
        Vector2d xy = diff.vec().rotated(-currentPose.getHeading());
        double heading = Angle.normDelta(targetPose.getHeading()) - Angle.normDelta(currentPose.getHeading());
        drive.setWeightedDrivePower(new Pose2d(xy,heading));
    }
}
