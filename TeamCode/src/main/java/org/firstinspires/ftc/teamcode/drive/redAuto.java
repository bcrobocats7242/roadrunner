package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
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

    double cX = 0;
    double cY = 0;
    double width = 0;

    OpenCvWebcam webcam1 = null;
    double noCube = 0;
    double leftThreshold = 500;
    double rightThreshold = 1000;

    public static final double objectWidthInRealWorldUnits = 3.75;
    public static final double focalLength = 728;

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
        Pose2d poseLeft = new Pose2d(7, -31.6, Math.toRadians(0));
        Pose2d leftForward = new Pose2d(30, -31.6, Math.toRadians(180));
        Pose2d poseCenter = new Pose2d(12, -33, Math.toRadians(270));
        Pose2d poseRight = new Pose2d(22.5, -38.6, Math.toRadians(270));
        Pose2d scoredLeft = new Pose2d(55.6, -26.5, Math.toRadians(180));
        Pose2d scoredCenter = new Pose2d(55.6, -33.5, Math.toRadians(180));
        Pose2d scoredRight = new Pose2d(55.6, -40, Math.toRadians(180));

        drive.setPoseEstimate(startPose);
        // STEP 1
        Trajectory LEFT_PURPLE_SCORE = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(7, -31.6, Math.toRadians(0)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        Trajectory LEFT_PURPLE_Forward = drive.trajectoryBuilder(poseLeft)
                .splineToLinearHeading(new Pose2d(30, -31.6, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        Trajectory CENTER_PURPLE_SCORE = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(12,-33, Math.toRadians(270)), Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        Trajectory RIGHT_PURPLE_SCORE = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(22.5, -38.6, Math.toRadians(270)), Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        // STEP 2
        Trajectory LEFT_YELLOW_SCORE = drive.trajectoryBuilder(leftForward)
                .splineToLinearHeading(new Pose2d(55.6, -26.5, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory CENTER_YELLOW_SCORE = drive.trajectoryBuilder(poseCenter)
                .splineToLinearHeading(new Pose2d(55.6, -33.5, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory RIGHT_YELLOW_SCORE = drive.trajectoryBuilder(poseRight)
                .splineToLinearHeading(new Pose2d(55.6, -40, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
       // STEP 3 Park
        Trajectory LEFT_PARK = drive.trajectoryBuilder(scoredLeft)
                .splineToLinearHeading(new Pose2d(51.6, -10.5, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory CENTER_PARK = drive.trajectoryBuilder(scoredCenter)
                .splineToLinearHeading(new Pose2d(51.6, -10.5, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory RIGHT_PARK = drive.trajectoryBuilder(scoredRight)
                .splineToLinearHeading(new Pose2d(51.6, -10.5, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        waitForStart();

        if (!isStopRequested()) {
            bucket.setPosition(0.7);
            drop.setPosition(0.15);
            if (cX < leftThreshold ) {
                telemetry.addLine("LEFT");
                drive.followTrajectory(LEFT_PURPLE_SCORE);
                drive.followTrajectory(LEFT_PURPLE_Forward);
                drive.followTrajectory(LEFT_YELLOW_SCORE);
                useSlide(1, 1350,-1300);
                drive.followTrajectory(LEFT_PARK);
            }
            else if (cX < rightThreshold && cX > leftThreshold) {
                telemetry.addLine("CENTER");
                drive.followTrajectory(CENTER_PURPLE_SCORE);
                drive.followTrajectory(CENTER_YELLOW_SCORE);
                useSlide(1, 1350,-1300);
                drive.followTrajectory(CENTER_PARK);
            }
            else if (cX > rightThreshold) {
                telemetry.addLine("RIGHT");
                drive.followTrajectory(RIGHT_PURPLE_SCORE);
                drive.followTrajectory(RIGHT_YELLOW_SCORE);
                useSlide(1, 1350,-1300);
                drive.followTrajectory(RIGHT_PARK);

            }


        }
    }

    public void useSlide(double pow, double dist, double powDown) {
        int encdist = Math.toIntExact(Math.round(dist));
        int encdistDown = Math.toIntExact(Math.round(dist- 150));


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
        sleep(500);
        bucket.setPosition(0.15);
        sleep(600);

        drop.setPosition(0.5);
        sleep(500);
        bucket.setPosition(0.35);
        drop.setPosition(0.15);
        sleep(300);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (powDown > 0) {
            slide.setTargetPosition(-encdistDown);
        } else {
            slide.setTargetPosition(encdistDown);
        }
        slide.setPower(powDown);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (slide.isBusy()) {
            telemetry.addData("busy", dist);
            telemetry.update();
        }
        slide.setPower(0);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    class RedCubePipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect yellow regions
            Mat redMask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(redMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest yellow contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);

                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                //Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

                if (cX < leftThreshold) {
                    telemetry.addLine("left");
                    telemetry.update();
                } else if (cX < rightThreshold && cX > leftThreshold) {
                    telemetry.addLine("middle");
                    telemetry.update();
                } else if (cX > rightThreshold) {
                    telemetry.addLine("right");
                    telemetry.update();
                }
                telemetry.update();

            }

            return input;
        }

        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Scalar lowerRed = new Scalar(100, 100, 100);
            Scalar upperRed = new Scalar(180, 255, 255);


            Mat redMask = new Mat();
            Core.inRange(hsvFrame, lowerRed, upperRed, redMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(redMask, redMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(redMask, redMask, Imgproc.MORPH_CLOSE, kernel);

            return redMask;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }
        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }

    }
    private static double getDistance(double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }
}