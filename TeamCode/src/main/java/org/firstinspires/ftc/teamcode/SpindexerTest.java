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

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Config
@TeleOp(name = "SpindexerTest")
public class SpindexerTest extends LinearOpMode {
    private DcMotor fL, bL, fR, bR; // carousel rotator will be fR
    private DcMotor intake, outtake, encoder;
    private Servo lights, linkage;
    private CRServo carouselRotator;
    private ColorSensor indicator2;
    private DistanceSensor distance;
    private float CurrentColor1, CurrentColor2;
    private final double ticks_in_degree = 700/180.0;
    private WebcamName cam;
    private int motif = 0;
    double targetDegrees = 0;
    boolean moving = false;
    boolean prevX = false;
    boolean prevB = false;
    public static double KP = 0.005;
    final double THRESH = 2.0; // degrees


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


        carouselRotator = hardwareMap.get(CRServo.class, "carouselRotator");
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
        // Rising/Falling Edge Detector
        Gamepad lastGamepad2 = new Gamepad();

        // Initial Positions
        linkage.setPosition(0.1);


        // April Tags | Check for the motif and set the rest to zero. | One-GPP; Two-PGP; Three PPG
        boolean motifOne = false, motifTwo = false;
        if (motif == 1) {
            motifOne = true;
        } else if (motif == 2) {
            motifTwo = true;
        }

        // Positions of each of section of the carousel | 0-Purple, 1-Green
        int pos1 = 0, pos2 = 0, pos3 = 0;
        boolean moving = false;
        double targetTicks = 0;
        boolean prevX = false;
        boolean prevB = false;
        final int TICKS_PER_REV = 8192;
        final int THRESH_TICKS = 50;
        final int TURN_TICKS = (int)(TICKS_PER_REV * (90.0 / 360.0)); // ≈ 2275 ticks

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                int position = encoder.getCurrentPosition();
                boolean x = gamepad2.x;
                boolean b = gamepad2.b;
                int currentTicks = encoder.getCurrentPosition() % TICKS_PER_REV;
                if (currentTicks < 0) currentTicks += TICKS_PER_REV;

                if (x && !prevX) {
                    targetTicks = (currentTicks + TURN_TICKS) % TICKS_PER_REV;
                    moving = true;
                }
                if (b && !prevB) {
                    targetTicks = (currentTicks - TURN_TICKS + TICKS_PER_REV) % TICKS_PER_REV;
                    moving = true;
                }
                double power = 0;
                double error = 0;

                if (moving) {
                    int delta = (int)(targetTicks - currentTicks);
                        // wrap error to shortest path (-4096..4096)
                    if (delta > TICKS_PER_REV / 2) delta -= TICKS_PER_REV;
                    if (delta < -TICKS_PER_REV / 2) delta += TICKS_PER_REV;
                    error = -delta;
                    power = KP * error;
                    power = Math.max(-1.0, Math.min(1.0, power));

                    if (Math.abs(error) < THRESH_TICKS) {
                        power = 0;
                        moving = false;
                    }
                }

                carouselRotator.setPower(power);

                telemetry.addData("Current ticks", currentTicks);
                telemetry.addData("Target ticks", targetTicks);
                telemetry.addData("Error (ticks)", error);
                telemetry.addData("Power", power);
                telemetry.addData("Moving", moving);
                telemetry.update();

                prevX = gamepad2.x;
                prevB = gamepad2.b;

                }
            }
        }
    }
