package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp(name = "FSM Test")
public class FSMTesting extends LinearOpMode {
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
    public enum intakeStates {
        Intake_START,
        Intake_TURN1,
        Intake_TURN2
    }

    intakeStates intakeState = intakeStates.Intake_START;

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

        linkage.setDirection(Servo.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        fR.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.REVERSE);
        double maxSpeed = 0.8;




        double f, r, s;
        double fLeftPower, bLeftPower, fRightPower, bRightPower;

        waitForStart();
        // Initial Positions
        linkage.setPosition(0.4);
        carouselRotator.setPosition(0.9);

        // Positions of each of section of the carousel | 0-Purple, 1-Green


        if (opModeIsActive()) {
            while (opModeIsActive()) {
                double distanceval = distance.getDistance(DistanceUnit.MM);
                telemetry.addData("Distance", distanceval);
                telemetry.addData("State", intakeState);
                telemetry.update();
                switch (intakeState) {
                    case Intake_START:
                        carouselRotator.setPosition(0.9);
                        if (distanceval < 90 && distanceval > 50) {
                            carouselRotator.setPosition(0.53);
                            intakeState = intakeStates.Intake_TURN1;
                            sleep(1500);

                        }
                        break;
                    case Intake_TURN1:
                        if (distanceval < 90) {
                            timer.reset();
                            carouselRotator.setPosition(0.16);
                            intakeState = intakeStates.Intake_TURN2;
                        }
                    case Intake_TURN2:
                        break;
                }

                if (gamepad2.x) {
                    intakeState = intakeStates.Intake_START;
                }
//                switch (outtakeState) {
//                    case Outtake_ROT1:
//                        if (gamepad2.y) {
//                            outtake.setPower(0.8);
//                            carouselRotator.setPosition(0.08);
//                            outtakeState = outtakeStates.Outtake_EXT1;
//                        }
//                        break;
//                    case Outtake_EXT1:
//                        linkage.setPosition(0.15);
//                        sleep(500);
//                        linkage.setPosition(0.4);
//                        outtakeState = outtakeStates.Outtake_ROT2;
//                        break;
//                    case Outtake_ROT2:
//                        carouselRotator.setPosition(0.45);
//                        outtakeState = outtakeStates.Outtake_EXT2;
//                        break;
//                    case Outtake_EXT2:
//                        linkage.setPosition(0.15);
//                        sleep(500);
//                        linkage.setPosition(0.4);
//                        outtakeState = outtakeStates.Outtake_ROT3;
//                        break;
//                    case Outtake_ROT3:
//                        carouselRotator.setPosition(0.08);
//                        outtakeState = outtakeStates.Outtake_EXT3;
//                        break;
//                    case Outtake_EXT3:
//                        linkage.setPosition(0.15);
//                        sleep(500);
//                        linkage.setPosition(0.4);
//                        outtakeState = outtakeStates.Outtake_ROT1;
//                        intakeState = intakeStates.Intake_START;
//                        break;
//                    }
//                if (gamepad2.a) {
//                    outtakeState = outtakeStates.Outtake_ROT1;
//                }

                if (gamepad2.y) {
                    outtake.setPower(0.8);
                    sleep(1000);
                    carouselRotator.setPosition(0.08);
                    sleep(500);
                    linkage.setPosition(0.15);
                    sleep(500);
                    linkage.setPosition(0.4);
                    sleep(500);
                    carouselRotator.setPosition(0.45);
                    sleep(500);
                    linkage.setPosition(0.15);
                    sleep(500);
                    linkage.setPosition(0.4);
                    sleep(500);
                    carouselRotator.setPosition(0.82);
                    sleep(500);
                    linkage.setPosition(0.15);
                    sleep(500);
                    linkage.setPosition(0.4);
                    sleep(500);
                    outtake.setPower(0);
                    intakeState = intakeStates.Intake_START;
                }
            }
        }
    }
}
