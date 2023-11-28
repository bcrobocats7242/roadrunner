package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class redAuto extends LinearOpMode {
    DcMotor slide;
    Servo bucket;
    Servo drop;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        slide = hardwareMap.get(DcMotor.class, "slide");
        bucket = hardwareMap.get(Servo.class,"bucket");
        drop = hardwareMap.get(Servo.class,"drop");


        Pose2d startPose = new Pose2d(12, -61, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(54, -38.4, Math.toRadians(180)), Math.toRadians(0))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(startPose)
                .back(60)
                .build();

        waitForStart();

        if (!isStopRequested()) {
            bucket.setPosition(0.65);
            drop.setPosition(0);
            drive.followTrajectory(traj1);
            useSlide(1, 1250);
            drive.followTrajectory(traj2);

        }
    }
    public void useSlide (double pow, double dist) {
        int encdist = Math.toIntExact(Math.round(dist));

        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (pow > 0) {
            slide.setTargetPosition(-encdist);
        } else {
            slide.setTargetPosition(encdist);
        }
        slide.setPower(pow);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (slide.isBusy()) {
            telemetry.addData("busy", dist);
            telemetry.update();
        }
        sleep(380);
        bucket.setPosition(0.5);
        sleep(300);

        drop.setPosition(0.5);
        sleep(500);
        bucket.setPosition(0.65);
        drop.setPosition(0);
        sleep(500);
    //    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    //    sleep(500);
        slide.setPower(0);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

}