package org.firstinspires.ftc.teamcode;

import android.graphics.Camera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@TeleOp(name = "MechaTeleOp")
public class MechaTeleOp extends LinearOpMode {
    private DcMotor fL, bL, fR, bR; // carousel rotator will be fR
    private DcMotor intake, outtake;
    private Servo lights, linkage;
    private CRServo carouselRotator;
    private ColorSensor indicator1, indicator2;
    private float CurrentColor1, CurrentColor2;
    private final double ticks_in_degree = 700/180.0;
    private WebcamName cam;
    private int motif = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        cam = hardwareMap.get(WebcamName.class, "cam");
        fL = hardwareMap.dcMotor.get("fL");
        bL = hardwareMap.dcMotor.get("bL");
        fR = hardwareMap.dcMotor.get("fR");
        bR = hardwareMap.dcMotor.get("bR");
        intake = hardwareMap.dcMotor.get("intake");
        outtake = hardwareMap.dcMotor.get("outtake");


        carouselRotator = hardwareMap.get(CRServo.class, "carouselRotator");
        linkage = hardwareMap.get(Servo.class, "linkage");
        lights = hardwareMap.get(Servo.class, "lights");
        indicator1 = hardwareMap.colorSensor.get("indicator1");
        indicator2 = hardwareMap.colorSensor.get("indicator2");


        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setDirection(DcMotor.Direction.REVERSE);
        fR.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.REVERSE);
        double maxSpeed = 0.8;
        double degrees = fR.getCurrentPosition() * (360.0/8192.0);

        while (!isStarted() && !isStopRequested()) {

            // Increase number with D-Pad Up
            if (gamepad1.dpad_up) {
                motif++;
                sleep(200);  // debounce delay
            } else if (gamepad1.dpad_down) {
                motif--;
                sleep(200); //debounce delay
            }

            telemetry.addData("Motif", motif);
        }


        double f, r, s;
        double fLeftPower, bLeftPower, fRightPower, bRightPower;

        waitForStart();
        // Rising/Falling Edge Detector
        Gamepad lastGamepad2 = new Gamepad();

        // Initial Positions
        linkage.setPosition(0);
        moveToPosition(0);

        // April Tags | Check for the motif and set the rest to zero. | One-GPP; Two-PGP; Three PPG
        boolean motifOne = false, motifTwo = false;
        if (motif == 1) {
            motifOne = true;
        } else if (motif == 2) {
            motifTwo = true;
        }

        // Positions of each of section of the carousel | 0-Purple, 1-Green
        int pos1 = 0, pos2 = 0, pos3 = 0;




        if (opModeIsActive()) {
            while (opModeIsActive()) {
                CurrentColor1 = JavaUtil.rgbToHue(indicator1.red(), indicator1.green(), indicator1.blue());
                CurrentColor2 = JavaUtil.rgbToHue(indicator2.red(), indicator2.green(), indicator2.blue());
                telemetry.addData("Sensor 1 Hue", CurrentColor1);
                telemetry.addData("Sensor 2 Hue", CurrentColor2);
                telemetry.update();
                if (CurrentColor1 > 240 && CurrentColor1 < 360) {
                    lights.setPosition(0.715);
                    pos1 = 0;
                } else if (CurrentColor1 > 60 && CurrentColor1 < 240) {
                    lights.setPosition(0.5);
                    pos1 = 1;
                } else {
                    lights.setPosition(0.279);
                }
                if (CurrentColor2 > 240 && CurrentColor2 < 360) {
                    pos2 = 0;
                } else if (CurrentColor2 > 60 && CurrentColor2 < 240) {
                    pos2 = 1;
                }

//                if (gamepad2.right_bumper) {
                    // Color Determination Algorithm
                    // If you have GPP
//                    if (pos1 == 1 && pos2 == 0) {
//                        pos3 = 0;
//                        if (motifOne) {
//                            autoShoot();
//                        } else if (motifTwo) {
//                            outtake.setPower(1);
//                            carouselRotator.setPosition(0.3);
//                            shoot();
//                            carouselRotator.setPosition(0);
//                            shoot();
//                            carouselRotator.setPosition(0.6);
//                            shoot();
//                            carouselRotator.setPosition(0);
//                            sleep(200);
//                            outtake.setPower(0);
//                        } else {
//                            outtake.setPower(1);
//                            carouselRotator.setPosition(0.3);
//                            shoot();
//                            carouselRotator.setPosition(0.6);
//                            shoot();
//                            carouselRotator.setPosition(0);
//                            shoot();
//                            sleep(200);
//                            outtake.setPower(0);
//
//                        }
//                        // If you have PGP
//                    } else if (pos1 == 0 && pos2 == 1) {
//                        pos3 = 0;
//                        if (motifOne) {
//                            outtake.setPower(1);
//                            carouselRotator.setPosition(0.3);
//                            shoot();
//                            carouselRotator.setPosition(0.6);
//                            shoot();
//                            carouselRotator.setPosition(0);
//                            shoot();
//                            sleep(200);
//                            outtake.setPower(0);
//                        } else if (motifTwo) {
//                            autoShoot();
//                        } else {
//                            outtake.setPower(1);
//                            shoot();
//                            carouselRotator.setPosition(0.6);
//                            shoot();
//                            carouselRotator.setPosition(0.3);
//                            shoot();
//                            carouselRotator.setPosition(0);
//                            sleep(200);
//                            outtake.setPower(0);
//                        }
//                        // If you have PPG
//                    } else if (pos1 == 0 && pos2 == 0) {
//                        pos3 = 1;
//                        if (motifOne) {
//                            outtake.setPower(1);
//                            carouselRotator.setPosition(0.6);
//                            shoot();
//                            carouselRotator.setPosition(0.3);
//                            shoot();
//                            carouselRotator.setPosition(0);
//                            shoot();
//                            sleep(200);
//                            outtake.setPower(0);
//
//                        } else if (motifTwo) {
//                            outtake.setPower(1);
//                            shoot();
//                            carouselRotator.setPosition(0);
//                            shoot();
//                            carouselRotator.setPosition(0.6);
//                            shoot();
//                            carouselRotator.setPosition(0.3);
//                            sleep(200);
//                            outtake.setPower(0);
//
//                        } else {
//                            autoShoot();
//                        }
//                        // Accidental non-pattern
//                    } else {
//                        autoShoot();
//                    }
//                }

                if (gamepad2.left_bumper) {
                    outtake.setPower(0.17);
                }  else if (gamepad2.a) {
                    outtake.setPower(0.5);
                } else if (gamepad2.y) {
                    outtake.setPower(1);
                } else {
                    outtake.setPower(0);
                }


                // Manual Methods
                //Intake
                if (gamepad1.a) {
                    intake.setPower(1);
                } else {
                    intake.setPower(0);

                }
                // Linkage Gamepad 2
//                if (gamepad2.a) {
//                    linkage.setPosition(0);
//                } else if (gamepad2.y) {
//                    linkage.setPosition(0.6);
//                }
//                // Carousel Rotator
                if (gamepad2.x && !lastGamepad2.x) {
                    moveToPosition(degrees + 120.0);
                }
                if (gamepad2.b && !lastGamepad2.b) {
                    moveToPosition(degrees-120.0);
                }
                



                f = gamepad1.left_stick_y;
                r = -gamepad1.right_stick_x;
                s = -gamepad1.left_stick_x;
                fLeftPower = (f + r + s);
                bLeftPower = (f + r - s);
                fRightPower = (f - r - s);
                bRightPower = (f - r + s);
                double maxN = Math.max(Math.abs(fLeftPower), Math.max(Math.abs(bLeftPower),
                        Math.max(Math.abs(fRightPower), Math.abs(bRightPower))));
                if (maxN > 1) {
                    fLeftPower /= maxN;
                    bLeftPower /= maxN;
                    fRightPower /= maxN;
                    bRightPower /= maxN;
                }
                fL.setPower(fLeftPower * maxSpeed);
                bL.setPower(bLeftPower * maxSpeed);
                fR.setPower(fRightPower * maxSpeed);
                bR.setPower(bRightPower * maxSpeed);
            }
        }
    }

    private void moveToPosition(double targetDegrees) throws InterruptedException {
        double kP = 0.005;
        double currentDegrees = fR.getCurrentPosition()*(360.0/8192.0);
        double error = targetDegrees-currentDegrees;
        double power = kP*error;
        power = Math.max(-1.0, Math.min(1.0, power));
        if (Math.abs(error) < 2.0) {
            power = 0;
        }
        carouselRotator.setPower(power);
    }

    private void sort() throws InterruptedException {
    }
    private void shoot() throws InterruptedException {
        linkage.setPosition(0.5);
        sleep(500);
        linkage.setPosition(0);
    }
}

