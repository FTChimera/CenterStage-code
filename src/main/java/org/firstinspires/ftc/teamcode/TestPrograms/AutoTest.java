package org.firstinspires.ftc.teamcode.TestPrograms;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;



import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Autonomous
public class AutoTest extends LinearOpMode {

    SampleMecanumDrive drivetrain;
    DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
    Pose2d pose = new Pose2d();

    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        // 0.00071 kA , 0.04774 kV,  0.19851 kS
        drivetrain = new SampleMecanumDrive(hardwareMap);
        drivetrain.setPoseEstimate(pose);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()){

            drive(5,0.4,Direction.RIGHT);
            sleep(10);
            drive(5,0.4,Direction.LEFT);
            // drivetrain.turn(Math.toRadians(90));
            telemetry.addData("Y",pose.getY());
            telemetry.update();

        }

        return ;
    }
    private void drive(double inches, double power, Direction direction){
        switch (direction){
            case FORWARD:
                while (pose.getX() < inches && opModeIsActive() ){
                    setMotorPowers(power);
                    drivetrain.update();
                    pose = drivetrain.getPoseEstimate();

                }
                setMotorPowers(0);
                break;
            case BACKWARD:
                while (pose.getX() > -inches && opModeIsActive() ){
                   setMotorPowers(-power);
                    drivetrain.update();
                    pose = drivetrain.getPoseEstimate();
                }
                setMotorPowers(0);
                break;
            case RIGHT:
                while (pose.getY() > -inches && opModeIsActive()){
                    motorFrontLeft.setPower(power);
                    motorFrontRight.setPower(-power);
                    motorBackLeft.setPower(-power);
                    motorBackRight.setPower(power);
                    drivetrain.update();
                    pose = drivetrain.getPoseEstimate();
                    telemetry.addData("Y",pose.getY());
                    telemetry.update();
                }
                setMotorPowers(0);
                break;
            case LEFT:
                while (pose.getY() < inches && opModeIsActive()) {
                    motorFrontLeft.setPower(-power);
                    motorFrontRight.setPower(power);
                    motorBackLeft.setPower(power);
                    motorBackRight.setPower(-power);
                    drivetrain.update();
                    pose = drivetrain.getPoseEstimate();
                }
                setMotorPowers(0);
                break;




        }
        pose = new Pose2d();
        drivetrain.setPoseEstimate(pose);

    }
    private void setMotorPowers(double power){
        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorBackLeft.setPower(power);
        motorBackRight.setPower(power);
    }



}
enum Direction{
    FORWARD,BACKWARD,RIGHT,LEFT
}