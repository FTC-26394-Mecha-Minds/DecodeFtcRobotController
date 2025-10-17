package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "MechaTeleOp")
public class MechaTeleOp extends LinearOpMode {
    private DcMotor fL, bL, fR, bR;
    private DcMotor outtake;
    private Servo lights, carouselRotatator, linkage;
    private ColorSensor indicator;
    private float CurrentColor;
    private final double ticks_in_degree = 700/180.0;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        fL = hardwareMap.dcMotor.get("fL");
        bL = hardwareMap.dcMotor.get("bL");
        fR = hardwareMap.dcMotor.get("fR");
        bR = hardwareMap.dcMotor.get("bR");


        lights = hardwareMap.get(Servo.class, "lights");
        indicator = hardwareMap.colorSensor.get("indicator");


        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        fR.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.REVERSE);
        double maxSpeed = 0.8;


        double f, r, s;
        double fLeftPower, bLeftPower, fRightPower, bRightPower;

        waitForStart();
        // April Tags
        int motifZero = 0, motifOne = 1, motifTwo = 2;

        // Positions of each of section of the carousel | 0-Purple, 1-Green
        int pos1 = 0, pos2 = 0, pos3 = 0;




        if (opModeIsActive()) {
            while (opModeIsActive()) {
                CurrentColor = JavaUtil.rgbToHue(indicator.red(), indicator.green(), indicator.blue());
                telemetry.addData("Hue", CurrentColor);
                telemetry.update();
                if (CurrentColor > 240 && CurrentColor < 360) {
                    lights.setPosition(0.715);
                } else if (CurrentColor > 60 && CurrentColor < 240) {
                    lights.setPosition(0.5);
                } else {
                    lights.setPosition(0.279);
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

