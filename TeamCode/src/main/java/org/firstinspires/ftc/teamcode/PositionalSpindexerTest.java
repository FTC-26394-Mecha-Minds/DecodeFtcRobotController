package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Config
@TeleOp(name = "PositionalSpindexerTest")
public class PositionalSpindexerTest extends LinearOpMode {
    private DcMotor fL, bL, fR, bR; // carousel rotator will be fR
    private DcMotor intake, outtake, encoder;
    private Servo lights, linkage;
    private Servo carouselRotator;
    private ColorSensor indicator2;
    private DistanceSensor distance;
    private float CurrentColor2;
    private final double ticks_in_degree = 700/180.0;
    private WebcamName cam;
    private int motif = 0;
    public static double pos = 0.0, posOne = 0.0, outtakePower = 0;


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
        encoder = hardwareMap.dcMotor.get("encoder");


        carouselRotator = hardwareMap.get(Servo.class, "carouselRotator");
        linkage = hardwareMap.get(Servo.class, "linkage");
        lights = hardwareMap.get(Servo.class, "lights");
        distance = hardwareMap.get(DistanceSensor.class, "distance");
        indicator2 = hardwareMap.colorSensor.get("indicator2");


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



        double f, r, s;
        double fLeftPower, bLeftPower, fRightPower, bRightPower;

        waitForStart();
        // Rising/Falling Edge Detector
        Gamepad lastGamepad2 = new Gamepad();

        // Initial Positions
        linkage.setPosition(0.4);
        carouselRotator.setPosition(0.45);


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
                double position = linkage.getPosition();
                telemetry.addData("Position", position);
                if (gamepad2.left_bumper) {
                    outtake.setPower(outtakePower);
                } else {
                    outtake.setPower(0);
                }
                if (gamepad2.a) {
                    linkage.setPosition(0.4);
                } else if (gamepad2.y) {
                    linkage.setPosition(0.15);
                }
                carouselRotator.setPosition(pos);
            }
        }
    }
}
