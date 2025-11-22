package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Config
@TeleOp
public class ColorSortingAlgorithm extends LinearOpMode {
    private DcMotor fL, bL, fR, bR; // carousel rotator will be fR
    private DcMotor intake, outtake, encoder;
    private Servo lights, linkage;
    private Servo carouselRotator;
    private ColorSensor indicator2;
    private DistanceSensor distance;
    private float CurrentColor1, CurrentColor2;
    private final double ticks_in_degree = 700/180.0;
    private WebcamName cam;
    private int motif = 0;
    public static boolean motif1GPP = false, motif2PGP = false;
    public int pos1 = 0, pos2 = 0, pos3 = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        cam = hardwareMap.get(WebcamName.class, "cam");
        fL = hardwareMap.dcMotor.get("fL");
        bL = hardwareMap.dcMotor.get("bL");
        fR = hardwareMap.dcMotor.get("fR");
        bR = hardwareMap.dcMotor.get("bR");
        intake = hardwareMap.dcMotor.get("intake");
        outtake = hardwareMap.dcMotor.get("outtake");
        encoder = hardwareMap.dcMotor.get("encoder");


        carouselRotator = hardwareMap.get(Servo.class, "carouselRotator");
        linkage = hardwareMap.get(Servo.class, "linkage");
        lights = hardwareMap.get(Servo.class, "lights");
        indicator2 = hardwareMap.colorSensor.get("indicator2");
        distance = hardwareMap.get(DistanceSensor.class, "distance");



        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setDirection(DcMotor.Direction.REVERSE);
        fR.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.REVERSE);
        double maxSpeed = 0.8;




        double f, r, s;
        double fLeftPower, bLeftPower, fRightPower, bRightPower;

        waitForStart();
        // Initial Positions
        carouselRotator.setPosition(0.53); // Shooting Position = 0.45, 0.82, 0.08

        // Positions of each of section of the carousel | 0-Purple, 1-Green


        if (opModeIsActive()) {
            while (opModeIsActive()) {
                CurrentColor2 = JavaUtil.rgbToHue(indicator2.red(), indicator2.green(), indicator2.blue());
                telemetry.addData("Current Color", CurrentColor2);
                telemetry.addData("Position 1", pos1);
                telemetry.addData("Position 2", pos2);
                if (gamepad2.a) {
                    check(pos2);
//                    if (CurrentColor2 > 240 && CurrentColor2 < 360) {
//                        pos2 = 0;
//                    } else if (CurrentColor2 > 60 && CurrentColor2 < 240) {
//                        pos2 = 1;
//                    }
                    sleep(1100);
                    carouselRotator.setPosition(0.9);
                    check(pos1);
//                    if (CurrentColor2 > 240 && CurrentColor2 < 360) {
//                        pos1 = 0;
//                    } else if (CurrentColor2 > 60 && CurrentColor2 < 240) {
//                        pos1 = 1;
//                    }
                }
                if (gamepad2.x) {
                    carouselRotator.setPosition(0.9);
                    sleep(2500);
                    carouselRotator.setPosition(0.53);
                    sleep(2500);
                    carouselRotator.setPosition(0.16);
                    sleep(1500);
                    carouselRotator.setPosition(0.08);
                    sleep(1500);
                    carouselRotator.setPosition(0.45);
                    sleep(1500);
                    carouselRotator.setPosition(0.82);
                    sleep(1500);
                    carouselRotator.setPosition(0.9);

                }
                if (gamepad2.b) {
                    carouselRotator.setPosition(0.53);
                }

                if (gamepad2.right_bumper) {
//                 Color Determination Algorithm
//                 If you have GPP
                    if (pos1 == 1 && pos2 == 0) {
                        pos3 = 0;
                        if (motif1GPP) {
                            defaultSort();
                        } else if (motif2PGP) {
                            carouselRotator.setPosition(0.53);
                            sleep(3100);
                            carouselRotator.setPosition(0.9);
                            sleep(3100);
                            carouselRotator.setPosition(0.16);
                        } else {
                            carouselRotator.setPosition(0.53);
                            sleep(3100);
                            carouselRotator.setPosition(0.16);
                            sleep(3100);
                            carouselRotator.setPosition(0.9);

                        }
                        // If you have PGP
                    } else if (pos1 == 0 && pos2 == 1) {
                        pos3 = 0;
                        if (motif1GPP) {
                            carouselRotator.setPosition(0.16);
                            sleep(3100);
                            carouselRotator.setPosition(0.53);
                            sleep(3100);
                            carouselRotator.setPosition(0.9);
                            outtake.setPower(0);
                        } else if (motif2PGP) {
                            carouselRotator.setPosition(0.9);
                            sleep(3100);
                            carouselRotator.setPosition(0.16);
                            sleep(3100);
                            carouselRotator.setPosition(0.53);
                        } else {
                            defaultSort();
                        }
                        // If you have PPG
                    } else if (pos1 == 0 && pos2 == 0) {
                        pos3 = 1;
                        if (motif1GPP) {
                            carouselRotator.setPosition(0.53);
                            sleep(3100);
                            carouselRotator.setPosition(0.9);
                            sleep(3100);
                            carouselRotator.setPosition(0.16);
                        } else if (motif2PGP) {
                            defaultSort();
                        } else {
                            carouselRotator.setPosition(0.9);
                            sleep(3100);
                            carouselRotator.setPosition(0.16);
                            sleep(3100);
                            carouselRotator.setPosition(0.53);
                        }
                        // Accidental non-pattern
                    }
                }
            }
        }
    }
    private void defaultSort() throws InterruptedException {
        carouselRotator.setPosition(0.9);
        sleep(3100);
        carouselRotator.setPosition(0.53);
        sleep(3100);
        carouselRotator.setPosition(0.16);
    }
    private void check(int pos) throws InterruptedException {
            if (CurrentColor2 > 240 && CurrentColor2 < 360) {
                pos = 0;
            } else if (CurrentColor2 > 60 && CurrentColor2 < 240) {
                pos = 1;
            }
    }
}
