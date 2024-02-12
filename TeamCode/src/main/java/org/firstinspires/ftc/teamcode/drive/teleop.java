package org.firstinspires.ftc.teamcode.drive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.security.Policy;

@TeleOp(name = "teleop")
public class teleop extends LinearOpMode {

    public DcMotor FL;
    public DcMotor BL;
    public DcMotor FR;
    public DcMotor BR;
    public DcMotor intake;
    public DcMotor slide;
    public DcMotor hangMotor;
    public Servo bucket;
    public Servo hangServo;
    public Servo launch;
    public Servo drop;

    @Override
    public void runOpMode() {
        float turn_FL_X = 0;
        float turn_BR_X = 0;
        float turn_FR_X = 0;
        float turn_BL_X = 0;
        float strafe_FR_X = 0;
        float strafe_BL_X = 0;
        float strafe_BR_X = 0;
        float strafe_FL_X = 0;
        float strafe_FL_Y = 0;
        float strafe_FR_Y = 0;
        float strafe_BL_Y = 0;
        float strafe_BR_Y = 0;
        double driveSpeed = 1.0;


        FL = hardwareMap.get(DcMotor.class, "FL");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");
        intake = hardwareMap.get(DcMotor.class, "intake");
        slide = hardwareMap.get(DcMotor.class,"slide");
        hangMotor = hardwareMap.get(DcMotor.class,"lift");
        bucket = hardwareMap.get(Servo.class,"bucket");
        hangServo = hardwareMap.get(Servo.class, "hangServo");
        launch = hardwareMap.get(Servo.class,"launch");
        drop = hardwareMap.get(Servo.class,"drop");

        waitForStart();
        if (opModeIsActive()) {
            FL.setDirection(DcMotorSimple.Direction.REVERSE);
            BL.setDirection(DcMotorSimple.Direction.REVERSE);
            FR.setDirection(DcMotorSimple.Direction.FORWARD);
            BR.setDirection(DcMotorSimple.Direction.FORWARD);
            FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            while (opModeIsActive()) {
                telemetry.addData("leftstickX", gamepad1.left_stick_x);
                telemetry.addData("leftstickY", gamepad1.left_stick_y);
                telemetry.addData("rightstickX", gamepad1.right_stick_x);
                telemetry.update();
                if (gamepad1.left_stick_y > 0.1) {
                    // forward
                    strafe_BR_Y = -gamepad1.left_stick_y;
                    strafe_FL_Y = -gamepad1.left_stick_y;
                    strafe_FR_Y = -gamepad1.left_stick_y;
                    strafe_BL_Y = -gamepad1.left_stick_y;
                } else if (gamepad1.left_stick_y < -0.1) {
                    // backward
                    strafe_BR_Y = -gamepad1.left_stick_y;
                    strafe_FL_Y = -gamepad1.left_stick_y;
                    strafe_FR_Y = -gamepad1.left_stick_y;
                    strafe_BL_Y = -gamepad1.left_stick_y;
                } else if (gamepad1.left_stick_x > 0.25) {
                    // right strafe
                    strafe_FL_X = gamepad1.left_stick_x;
                    strafe_FR_X = -gamepad1.left_stick_x;
                    strafe_BL_X = -gamepad1.left_stick_x;
                    strafe_BR_X = gamepad1.left_stick_x;
                } else if (gamepad1.left_stick_x < -0.25) {
                    // left strafe
                    strafe_FL_X = gamepad1.left_stick_x;
                    strafe_FR_X = -gamepad1.left_stick_x;
                    strafe_BL_X = -gamepad1.left_stick_x;
                    strafe_BR_X = gamepad1.left_stick_x;
                } else {
                    strafe_FL_X = 0;
                    strafe_FR_X = 0;
                    strafe_BL_X = 0;
                    strafe_BR_X = 0;
                    strafe_FL_Y = 0;
                    strafe_FR_Y = 0;
                    strafe_BL_Y = 0;
                    strafe_BR_Y = 0;
                }
                // turn
                if (gamepad1.right_stick_x > 0.1) {
                    // left turn
                    turn_FL_X = -gamepad1.right_stick_x;
                    turn_FR_X = gamepad1.right_stick_x;
                    turn_BL_X = -gamepad1.right_stick_x;
                    turn_BR_X = gamepad1.right_stick_x;
                } else if (gamepad1.right_stick_x < -0.1) {
                    // right turn
                    turn_FL_X = -gamepad1.right_stick_x;
                    turn_FR_X = gamepad1.right_stick_x;
                    turn_BL_X = -gamepad1.right_stick_x;
                    turn_BR_X = gamepad1.right_stick_x;
                }
                else {
                    turn_FL_X = 0;
                    turn_FR_X = 0;
                    turn_BL_X = 0;
                    turn_BR_X = 0;
                }
                // grab
                if (gamepad1.right_bumper) {
                    intake.setPower(1);
                    slide.setPower(0.1);
                } else if (gamepad1.left_bumper) {
                    intake.setPower(-1);
                } else {
                    intake.setPower(0);
                }
                // score
                if (gamepad1.x) {
                    bucket.setPosition(0.15);
                    if (gamepad1.y) {
                        drop.setPosition(0.5);
                    }}
                else if (gamepad1.b) {
                    bucket.setPosition(1);
                    if (gamepad1.y) {
                        drop.setPosition(0.5);
                    }}
                else {
                    bucket.setPosition(0.35);
                    drop.setPosition(0.15);
                }
                // drop
                // slide rail
                if (gamepad1.left_trigger > 0.1) {
                    slide.setPower(gamepad1.left_trigger);
                } else if (gamepad1.right_trigger > 0.1) {
                    slide.setPower(-gamepad1.right_trigger);
                } else {
                    slide.setPower(-0.1);
                 }
                // hang functions
                if (gamepad1.dpad_left) {
                    hangServo.setPosition(0);
                    hangMotor.setPower(-1); }
                else if (gamepad1.dpad_right) {
                    hangServo.setPosition(0);
                    hangMotor.setPower(1);
                }
                else if (gamepad1.dpad_down) {
                    hangServo.setPosition(0.28);
                }
                else if (gamepad1.dpad_up) {
                    hangServo.setPosition(0.63);
                }
                else {
                    hangMotor.setPower(0);
                    hangServo.close();
                }
                // launch drone
                if (gamepad1.right_stick_button && gamepad1.left_stick_button) {
                    launch.setPosition(1);
                } else {
                    launch.setPosition(0.9);
                }
                if (gamepad1.a && gamepad1.b) {
                    useSlide(1,1300,1100);
                }



                FL.setPower(driveSpeed * (turn_FL_X + strafe_FL_X + strafe_FL_Y));
                FR.setPower(driveSpeed * (turn_FR_X + strafe_FR_X + strafe_FR_Y));
                BL.setPower(driveSpeed * (turn_BL_X + strafe_BL_X + strafe_BL_Y));
                BR.setPower(driveSpeed * (turn_BR_X + strafe_BR_X + strafe_BR_Y));

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
}