package org.firstinspires.ftc.teamcode;

import android.graphics.Camera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "MechaTeleOp")
public class MechaTeleOp extends LinearOpMode {
    private DcMotor fL, bL, fR, bR; // carousel rotator will be fR
    private DcMotor intake, outtake, encoder;
    private Servo lights, linkage;
    private Servo carouselRotator;
    private ColorSensor indicator2;
    private DistanceSensor distance;
    private float CurrentColor2;
    private WebcamName cam;
    private int motif = 0;
    public enum intakeStates {
        Intake_START,
        Intake_DELAY1,
        Intake_TURN1,
        Intake_BUFFER,
        Intake_DELAY2,
        Intake_TURN2
    }
    public enum outtakeStates {
        Outtake_START,
        Outtake_TURN1,
        Outtake_SHOOT1_START,
        Outtake_SHOOT1_PULSE,
        Outtake_TURN2,
        Outtake_SHOOT2_START,
        Outtake_SHOOT2_PULSE,
        Outtake_TURN3,
        Outtake_SHOOT3_START,
        Outtake_SHOOT3_PULSE,
        Outtake_BUFFER,
        Outtake_DONE
    }
    intakeStates intakeState = intakeStates.Intake_START;
    outtakeStates outtakeState = outtakeStates.Outtake_START;



    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime timer = new ElapsedTime();
        ElapsedTime outtakeTimer = new ElapsedTime();
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
        linkage.setDirection(Servo.Direction.REVERSE);
        double maxSpeed = 0.8;
        boolean lastY = false;



        double f, r, s;
        double fLeftPower, bLeftPower, fRightPower, bRightPower;

        waitForStart();
        // Rising/Falling Edge Detector
        Gamepad lastGamepad2 = new Gamepad();

        // Initial Positions
        linkage.setPosition(0.4);
        carouselRotator.setPosition(0.9);



        if (opModeIsActive()) {
            while (opModeIsActive()) {
                CurrentColor2 = JavaUtil.rgbToHue(indicator2.red(), indicator2.green(), indicator2.blue());
                int position = encoder.getCurrentPosition();
                double distanceval = distance.getDistance(DistanceUnit.MM);
                telemetry.addData("Distance", distanceval);
                telemetry.addData("State", intakeState);
                telemetry.addData("Sensor 2 Hue", CurrentColor2);
                telemetry.addData("Encoder", position);
                telemetry.update();
                if (CurrentColor2 > 180 && CurrentColor2 < 360) {
                    lights.setPosition(0.715);
                } else if (CurrentColor2 > 60 && CurrentColor2 < 180) {
                    lights.setPosition(0.5);
                } else {
                    lights.setPosition(0.279);
                }
                boolean outtakeActive = outtakeState != outtakeStates.Outtake_START && outtakeState != outtakeStates.Outtake_DONE;
                switch (intakeState) {
                    case Intake_START:
                        if (!outtakeActive) carouselRotator.setPosition(0.9);
                        if (distanceval < 90 && distanceval > 50) {
                            timer.reset();   // start delay timer
                            intakeState = intakeStates.Intake_DELAY1;
                        }
                        break;
                    case Intake_DELAY1:
                        if (timer.milliseconds() > 300) {
                            if (!outtakeActive) carouselRotator.setPosition(0.53);
                            timer.reset();
                            intakeState = intakeStates.Intake_BUFFER;
                        }
                        break;
                    case Intake_BUFFER:
                        if (timer.milliseconds() > 250) {
                            intakeState = intakeStates.Intake_TURN1;
                        }
                        break;
                    case Intake_TURN1:
                        if (distanceval < 90) {
                            timer.reset();
                            intakeState = intakeStates.Intake_DELAY2;
                        }
                        break;
                    case Intake_DELAY2:
                        if (timer.milliseconds() > 300) {
                            if (!outtakeActive) carouselRotator.setPosition(0.16);
                            intakeState = intakeStates.Intake_TURN2;
                        }
                        break;
                    case Intake_TURN2:
                        break;
                }
                if (gamepad2.x) {
                    intakeState = intakeStates.Intake_START;
                }
                boolean yPressed = gamepad2.y;
                boolean yRisingEdge = yPressed && !lastY;
                lastY = yPressed;

                if (yRisingEdge && outtakeState == outtakeStates.Outtake_START) {
                    outtakeTimer.reset();
                    outtakeState = outtakeStates.Outtake_TURN1;
                }
                switch (outtakeState) {
                    case Outtake_TURN1:
                        if (outtakeTimer.milliseconds() > 200) {
                            carouselRotator.setPosition(0.08);
                            outtakeTimer.reset();
                            outtakeState = outtakeStates.Outtake_SHOOT1_START;
                        }
                        break;
                    case Outtake_SHOOT1_START:
                        if (outtakeTimer.milliseconds() > 500) {
                            linkage.setPosition(0.15);
                            outtakeTimer.reset();
                            outtakeState = outtakeStates.Outtake_SHOOT1_PULSE;
                        }
                        break;
                    case Outtake_SHOOT1_PULSE:
                        if (outtakeTimer.milliseconds() > 250) {
                            linkage.setPosition(0.45);
                            outtakeTimer.reset();
                            outtakeState = outtakeStates.Outtake_TURN2;
                        }
                        break;
                    case Outtake_TURN2:
                        if (outtakeTimer.milliseconds() > 500) {
                            carouselRotator.setPosition(0.45);
                            outtakeTimer.reset();
                            outtakeState = outtakeStates.Outtake_SHOOT2_START;
                        }
                        break;
                    case Outtake_SHOOT2_START:
                        if (outtakeTimer.milliseconds() > 500) {
                            linkage.setPosition(0.15);
                            outtakeTimer.reset();
                            outtakeState = outtakeStates.Outtake_SHOOT2_PULSE;
                        }
                        break;
                    case Outtake_SHOOT2_PULSE:
                        if (outtakeTimer.milliseconds() > 250) {
                            linkage.setPosition(0.45);
                            outtakeTimer.reset();
                            outtakeState = outtakeStates.Outtake_TURN3;
                        }
                        break;
                    case Outtake_TURN3:
                        if (outtakeTimer.milliseconds() > 500) {
                            carouselRotator.setPosition(0.82);
                            outtakeTimer.reset();
                            outtakeState = outtakeStates.Outtake_SHOOT3_START;
                        }
                        break;
                    case Outtake_SHOOT3_START:
                        if (outtakeTimer.milliseconds() > 500) {
                            linkage.setPosition(0.15);
                            outtakeTimer.reset();
                            outtakeState = outtakeStates.Outtake_SHOOT3_PULSE;
                        }
                        break;
                    case Outtake_SHOOT3_PULSE:
                        if (outtakeTimer.milliseconds() > 250) {
                            linkage.setPosition(0.4);
                            outtakeTimer.reset();
                            outtakeState = outtakeStates.Outtake_BUFFER;
                        }
                        break;
                    case Outtake_BUFFER:
                        if (outtakeTimer.milliseconds() > 100) {
                            intakeState = intakeStates.Intake_START;
                            outtakeState = outtakeStates.Outtake_DONE;
                        }
                        break;
                    case Outtake_DONE:
                        outtakeState = outtakeStates.Outtake_START;
                        break;
                }
                if (gamepad1.a) {
                    intake.setPower(1);
                } else {
                    intake.setPower(0);
                }
                if (gamepad2.dpad_up) {
                    outtake.setPower(0.8);
                }
                if (gamepad2.dpad_down) {
                    outtake.setPower(0);
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
}

