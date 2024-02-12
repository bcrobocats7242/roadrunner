package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
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
public class redAutoFarSeq extends LinearOpMode {
    DcMotor slide;
    Servo bucket;
    Servo drop;
    double cX = 0;
    double cY = 0;
    double width = 0;
    public static final double objectWidthInRealWorldUnits = 3.75;
    public static final double focalLength = 728;


    OpenCvWebcam webcam1 = null;
    boolean noCube = false;
    double leftThreshold = 500;
    double rightThreshold = 1000;
    @Override
    public void runOpMode() throws InterruptedException {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
        );
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam1.setPipeline(new redAutoFarSeq.RedCubePipeline());

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
        Pose2d startPose = new Pose2d(-35, -61, Math.toRadians(270));
        Pose2d leftScoredPose = new Pose2d(51, -30, Math.toRadians(180));
        Pose2d centerScoredPose = new Pose2d(52, -35, Math.toRadians(180));
        Pose2d rightScoredPose = new Pose2d(52, -40, Math.toRadians(180));


        drive.setPoseEstimate(startPose);

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(13)
                .lineToSplineHeading(new Pose2d(-47,-35, Math.toRadians(280)))
                .lineToSplineHeading(new Pose2d(-34,-50, Math.toRadians(270)))
                .back(30)
                .splineToSplineHeading(new Pose2d(20,-10, Math.toRadians(180)),Math.toRadians(15))
                .lineTo(new Vector2d(56, -28),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,
                        DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(13)
                .splineToSplineHeading(new Pose2d(-35,-33, Math.toRadians(270)), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(-50,-42, Math.toRadians(180)))
                .strafeRight(32)
                .back(64)
                .splineToConstantHeading(new Vector2d(55,-37), Math.toRadians(0),
                 SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(13)
                .splineToLinearHeading(new Pose2d(-33,-34, Math.toRadians(180)), Math.toRadians(90))
                .forward(10)
                .lineTo(new Vector2d(-39, -10))
                .back(70)
                .lineToConstantHeading(new Vector2d(55, -46),
                 SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
              .build();


        TrajectorySequence outOfWayLeft = drive.trajectorySequenceBuilder(leftScoredPose)
                .forward(10)
                .waitSeconds(1)
                .strafeRight(18)
                .build();
        TrajectorySequence outOfWayCenter = drive.trajectorySequenceBuilder(centerScoredPose)
                .forward(10)
                .waitSeconds(1)
                .strafeRight(23)
                .build();
        TrajectorySequence outOfWayRight = drive.trajectorySequenceBuilder(rightScoredPose)
                .forward(10)
                .waitSeconds(1)
                .strafeRight(25)
                .build();


        waitForStart();

        if (!isStopRequested()) {
            bucket.setPosition(0.7);
            drop.setPosition(0.15);
            if (cX < leftThreshold ) {
                drive.followTrajectorySequence(left);
                useSlide(1, 1350,-900);
                drive.followTrajectorySequence(outOfWayLeft);
            }
            else if (cX < rightThreshold && cX > leftThreshold) {
                drive.followTrajectorySequence(center);
                sleep(1000);
                useSlide(1, 1350, -900);
                drive.followTrajectorySequence(outOfWayCenter);
            }
            else if (cX > rightThreshold) {
                drive.followTrajectorySequence(right);
                useSlide(1, 1350, -900);
                drive.followTrajectorySequence(outOfWayRight);
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

        private static double getDistance(double width) {
            double distance = (objectWidthInRealWorldUnits * focalLength) / width;
            return distance;
        }
    }