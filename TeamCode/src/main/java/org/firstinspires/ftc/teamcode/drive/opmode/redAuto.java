package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class redAuto extends LinearOpMode {
    DcMotor slide;
    Servo bucket;
    Servo drop;

    OpenCvWebcam webcam1 = null;
    double lastDetectedcubeX = -1;
    double noCube = 0;
    double leftThreshold = 500;
    double rightThreshold = 1000;

    @Override
    public void runOpMode() throws InterruptedException {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
        );
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam1.setPipeline(new RedCubePipeline());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                webcam1.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int errorCode) {
            }
        });
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        slide = hardwareMap.get(DcMotor.class, "slide");
        bucket = hardwareMap.get(Servo.class, "bucket");
        drop = hardwareMap.get(Servo.class, "drop");


        Pose2d startPose = new Pose2d(12, -61, Math.toRadians(270));
        Pose2d poseLeft = new Pose2d(10, -31, Math.toRadians(0));
        Pose2d poseCenter = new Pose2d(12, -33, Math.toRadians(270));
        Pose2d poseRight = new Pose2d(27, -31, Math.toRadians(0));
        Pose2d scoredPose = new Pose2d(10, -31, Math.toRadians(0));

        drive.setPoseEstimate(startPose);
        // STEP 1 Score Purple
        Trajectory LEFT_PURPLE_SCORE = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(10, -31.6, Math.toRadians(0)), Math.toRadians(180))
                .build();
        Trajectory CENTER_PURPLE_SCORE = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(12,-33, Math.toRadians(270)), Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        Trajectory RIGHT_PURPLE_SCORE = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(27, -31.6, Math.toRadians(0)), Math.toRadians(180))
                .build();
        // STEP 2 Score Yellow
        Trajectory LEFT_YELLOW_SCORE = drive.trajectoryBuilder(poseLeft)
                .splineToLinearHeading(new Pose2d(52.6, -27.5, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory CENTER_YELLOW_SCORE = drive.trajectoryBuilder(poseCenter)
                .splineToLinearHeading(new Pose2d(54.6, -34.5, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory RIGHT_YELLOW_SCORE = drive.trajectoryBuilder(poseRight)
                .splineToLinearHeading(new Pose2d(52.6, -34, Math.toRadians(180)), Math.toRadians(0))
                .build();
        //STEP 3  Pickup White
        //   Trajectory PICK_UP_WHITE = drive.trajectoryBuilder(scoredPose)
        //      .build();
        //STEP 4 Score White

        waitForStart();

        if (!isStopRequested()) {
           bucket.setPosition(0.65);
           drop.setPosition(0);
            if (lastDetectedcubeX < leftThreshold ) {
                drive.followTrajectory(LEFT_PURPLE_SCORE);
                drive.followTrajectory(LEFT_YELLOW_SCORE);
            }
            else if (lastDetectedcubeX < rightThreshold && lastDetectedcubeX > leftThreshold) {
                drive.followTrajectory(CENTER_PURPLE_SCORE);
                drive.followTrajectory(CENTER_YELLOW_SCORE);

            }
            else if (lastDetectedcubeX > rightThreshold) {
                drive.followTrajectory(RIGHT_PURPLE_SCORE);
                drive.followTrajectory(RIGHT_YELLOW_SCORE);

            }
           useSlide(1, 1250);


        }
    }

    public void useSlide(double pow, double dist) {
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


    class RedCubePipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Define lower and upper bounds for pink in HSV color space
            Scalar lowerPink = new Scalar(140, 50, 50); // Adjust these values
            Scalar upperPink = new Scalar(170, 255, 255); // Adjust these values
            // Convert the input image to the HSV color space
            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
            // Create a binary mask to identify pink pixels
            Mat pinkMask = new Mat();
            Core.inRange(input, lowerPink, upperPink, pinkMask);
            // Apply morphological operations to reduce noise
            Mat morphKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(pinkMask, pinkMask, Imgproc.MORPH_CLOSE, morphKernel);
            Imgproc.morphologyEx(pinkMask, pinkMask, Imgproc.MORPH_OPEN, morphKernel);
            // Find contours in the binary mask
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(pinkMask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            // Filter the contours to select cube-like shapes
            List<MatOfPoint> cubeContours = new ArrayList<>();
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > 1000) { // Adjust this threshold based on your cube size
                    cubeContours.add(contour);
                }
            }
            // Process detected cube contours
            for (MatOfPoint cubeContour : cubeContours) {
                // Calculate the center point of the pink cube
                Moments moments = Imgproc.moments(cubeContour);
                double cubeX = moments.get_m10() / moments.get_m00();
                double centerY = moments.get_m01() / moments.get_m00();
                // Display telemetry for the pink cube's location
                telemetry.addData("Pink Cube X", cubeX);
                telemetry.addData("Pink Cube Y", centerY);
                telemetry.update();

                lastDetectedcubeX = cubeX;
                // Draw rectangles around the detected cubes on the original image
                Rect rect = Imgproc.boundingRect(cubeContour);
                Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(255, 0, 255), 2); // Pink color
            }
            return input;
        }
    }
    }