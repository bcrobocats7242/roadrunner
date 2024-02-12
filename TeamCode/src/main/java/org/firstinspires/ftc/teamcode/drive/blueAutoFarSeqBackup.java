package org.firstinspires.ftc.teamcode.drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
@Autonomous
public class blueAutoFarSeqBackup extends LinearOpMode {
    DcMotor slide;
    DcMotor intake;
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

        webcam1.setPipeline(new blueAutoFarSeqBackup.BlueCubePipeline());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                webcam1.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int errorCode) {
            }
        });
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        slide = hardwareMap.get(DcMotor.class, "slide");
        intake = hardwareMap.get(DcMotor.class, "intake");
        bucket = hardwareMap.get(Servo.class, "bucket");
        drop = hardwareMap.get(Servo.class, "drop");
        Pose2d startPose = new Pose2d(-35, 61, Math.toRadians(90));
        Pose2d centerPixel = new Pose2d( -60, 10, Math.toRadians(180));
        Pose2d leftScoredPose = new Pose2d(51, 46, Math.toRadians(180));
        Pose2d centerScoredPose = new Pose2d(52, 35, Math.toRadians(180));
        Pose2d rightScoredPose = new Pose2d(52, 22, Math.toRadians(180));


        drive.setPoseEstimate(startPose);

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-33,34, Math.toRadians(180)), Math.toRadians(90))
                .forward(10)
                .lineTo(new Vector2d(-39, 10))
                .back(70)
                .lineToConstantHeading(new Vector2d(55, 46))
                .build();

        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(-35,37, Math.toRadians(115)), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(-50,42, Math.toRadians(180)))
                .strafeLeft(32)
                //     .forward(12)
                .back(90)
                .lineToLinearHeading(new Pose2d(55, 40, Math.toRadians(180)))
                .build();

        //  TrajectorySequence center2 = drive.trajectorySequenceBuilder(centerPixel)
        //        .back(90)
        //        .lineToLinearHeading(new Pose2d(52, 35, Math.toRadians(180)))
        //          .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-46,36, Math.toRadians(80)))
                .lineToSplineHeading(new Pose2d(-34,50, Math.toRadians(90)))
                .forward(-30)
                .splineToSplineHeading(new Pose2d(20,10, Math.toRadians(180)),Math.toRadians(10))
                .lineTo(new Vector2d(55, 25))
                .build();


        TrajectorySequence outOfWayLeft = drive.trajectorySequenceBuilder(leftScoredPose)
                .strafeLeft(32)
                .build();
        TrajectorySequence outOfWayCenter = drive.trajectorySequenceBuilder(centerScoredPose)
                .strafeLeft(25)
                .build();
        TrajectorySequence outOfWayRight = drive.trajectorySequenceBuilder(rightScoredPose)
                .strafeLeft(18)
                .build();


        waitForStart();

        if (!isStopRequested()) {
            bucket.setPosition(0.7);
            drop.setPosition(0.15);
            if (cX < leftThreshold ) {
                drive.followTrajectorySequence(left);
                useSlide(1, 1350,-1300);
                drive.followTrajectorySequence(outOfWayLeft);

            }
            else if (cX < rightThreshold && cX > leftThreshold) {
                drive.followTrajectorySequence(center);
                //    useIntake();
                //    drive.followTrajectorySequence(center2);
                useSlide(1, 1350, -1300);
                drive.followTrajectorySequence(outOfWayCenter);
            }
            else if (cX > rightThreshold) {
                drive.followTrajectorySequence(right);
                useSlide(1, 1350, -1300);
                drive.followTrajectorySequence(outOfWayRight);
            }


        }
    }


    public void useSlide(double pow, double dist, double powDown) {
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
        sleep(500);
        bucket.setPosition(0.52);
        sleep(600);

        drop.setPosition(0.5);
        sleep(500);
        bucket.setPosition(0.7);
        drop.setPosition(0.15);

        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (powDown > 0) {
            slide.setTargetPosition(-encdist);
        } else {
            slide.setTargetPosition(encdist);
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

    public void useIntake() {
        intake.setPower(1);
        sleep(2000);
        intake.setPower(-1);
        sleep(500);
        intake.setPower(0);

    }

    class BlueCubePipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect yellow regions
            Mat blueMask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(blueMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest yellow contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a blue outline around the largest detected object
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

            Scalar lowerBlue = new Scalar(10, 50, 50);
            Scalar upperBlue = new Scalar(80, 255, 255);


            Mat blueMask = new Mat();
            Core.inRange(hsvFrame, lowerBlue, upperBlue, blueMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(blueMask, blueMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(blueMask, blueMask, Imgproc.MORPH_CLOSE, kernel);

            return blueMask;
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