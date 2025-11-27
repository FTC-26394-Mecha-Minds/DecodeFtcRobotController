package org.firstinspires.ftc.teamcode.autonomous;


import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.MechaTeleOp;

@Autonomous (name = "RedClassifierAuto", group = "Autonomous", preselectTeleOp = "MechaTeleOp")
public class RedClassifierAuto extends LinearOpMode {
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

    public class IntakeSpindex {
        private DcMotorEx intake, encoder;
        private Servo carouselRotator, lights;
        private ColorSensor indicator2;
        ElapsedTime timer = new ElapsedTime();
        private DistanceSensor distance;
        private WebcamName cam;

        public IntakeSpindex(HardwareMap hardwareMap) {
            intake = hardwareMap.get(DcMotorEx.class, "intake");
            intake.setDirection(DcMotorEx.Direction.REVERSE);
            distance = hardwareMap.get(DistanceSensor.class, "distance");
            carouselRotator = hardwareMap.get(Servo.class, "carouselRotator");
            lights = hardwareMap.get(Servo.class, "lights");
            indicator2 = hardwareMap.colorSensor.get("indicator2");
            cam = hardwareMap.get(WebcamName.class, "cam");
            encoder = hardwareMap.get(DcMotorEx.class, "encoder");
        }
        public class intakeProc implements Action {
            @Override

            public boolean run(@NonNull TelemetryPacket packet) {
                double distanceval = distance.getDistance(DistanceUnit.MM);
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
                            if (timer.milliseconds() > 700) {
                                intakeState = intakeStates.Intake_DELAY2;
                                timer.reset();
                            } else {
                                timer.reset();
                                intakeState = intakeStates.Intake_BUFFER;
                            }
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
                        return false;
                }
                return true;
            }
        }
        public Action intakeProc() {
            return new intakeProc();
        }
        public class intakeRun implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(1);
                return false;
            }
        }
        public Action intakeRun() {
            return new intakeRun();
        }
        public class intakeStop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(0);
                return false;
            }
        }
        public Action intakeStop() {
            return new intakeStop();
        }
        public class lightsOn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                lights.setPosition(0.279);
                return false;
            }
        }
        public Action lightsOn() {
            return new lightsOn();
        }
    }
    public class Outtake {
        private Servo linkage, carouselRotator;
        private DcMotorEx outtake;
        private VoltageSensor battery;
        ElapsedTime outtakeTimer = new ElapsedTime();
        double idealVoltage = 13.5;
        double outtakePower = 0.775;

        public Outtake(HardwareMap hardwareMap) {
            outtake = hardwareMap.get(DcMotorEx.class, "outtake");
            battery = hardwareMap.voltageSensor.iterator().next();
            linkage = hardwareMap.get(Servo.class, "linkage");
            carouselRotator = hardwareMap.get(Servo.class, "carouselRotator");
            linkage.setDirection(Servo.Direction.REVERSE);

        }
        public class ShootingSequence implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (outtakeState == outtakeStates.Outtake_START) {
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
                            linkage.setPosition(0.4);
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
                            linkage.setPosition(0.4);
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
                            outtakeState = outtakeStates.Outtake_DONE;
                        }
                        break;
                    case Outtake_DONE:
                        intakeState = intakeStates.Intake_START;
                        outtakeState = outtakeStates.Outtake_START;
                        return false;
                }
                return true;
            }
        }
        public Action ShootingSequence() {
            return new ShootingSequence();
        }
        public class outtakeRun implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double batteryVoltage = battery.getVoltage();
                double power = outtakePower*(idealVoltage/batteryVoltage);
                outtake.setPower(0.8);
                return false;
            }
        }
        public Action outtakeRun() {
            return new outtakeRun();
        }
        public class outtakeStop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtake.setPower(0);
                return false;
            }
        }
        public Action outtakeStop() {
            return new outtakeStop();
        }

    }



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d initialPose = new Pose2d(-62, 36, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        IntakeSpindex intakeSpindex = new IntakeSpindex(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        waitForStart();

        TrajectoryActionBuilder shootPreload = drive.actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(-22, 16, Math.toRadians(130)),0)
                .afterTime(0, outtake.ShootingSequence())
                .waitSeconds(4.5);
        TrajectoryActionBuilder shootOne = drive.actionBuilder(new Pose2d(-12, 48, Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(-22, 16, Math.toRadians(130)),0)
                .afterTime(0, outtake.ShootingSequence())
                .waitSeconds(4.5);
        TrajectoryActionBuilder shootTwo = drive.actionBuilder(new Pose2d(12, 48, Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(-22, 16, Math.toRadians(130)),0)
                .afterTime(0, outtake.ShootingSequence())
                .waitSeconds(4.5);
        TrajectoryActionBuilder intakeOne = drive.actionBuilder(new Pose2d(-22, 16, Math.toRadians(140)))
                .splineToLinearHeading(new Pose2d(-12, 34, Math.toRadians(270)), 0)
                .waitSeconds(0.5)
                .lineToYConstantHeading(38)
                .waitSeconds(0.5)
                .lineToYConstantHeading(48)
                .waitSeconds(0.5);
        TrajectoryActionBuilder intakeTwo = drive.actionBuilder(new Pose2d(-22, 16, Math.toRadians(140)))
                .splineToLinearHeading(new Pose2d(12, 33, Math.toRadians(270)), 0)
                .waitSeconds(0.5)
                .lineToYConstantHeading(38)
                .waitSeconds(0.5)
                .lineToYConstantHeading(48)
                .waitSeconds(0.5);


        if (isStopRequested()) {return;}
        Actions.runBlocking(
                new ParallelAction(
                    intakeSpindex.lightsOn(),
                    new SequentialAction(
                            outtake.outtakeRun(),
                            shootPreload.build(),
                            outtake.outtakeStop(),
                            new ParallelAction(
                                    intakeSpindex.intakeRun(),
                                    intakeOne.build(),
                                    intakeSpindex.intakeProc()
                            ),
                            intakeSpindex.intakeStop(),
                            outtake.outtakeRun(),
                            shootOne.build(),
                            outtake.outtakeStop(),
                            new ParallelAction(
                                    intakeSpindex.intakeRun(),
                                    intakeTwo.build(),
                                    intakeSpindex.intakeProc()
                            ),
                            intakeSpindex.intakeStop(),
                            outtake.outtakeRun(),
                            shootTwo.build(),
                            outtake.outtakeStop()
                    )
                )
        );


    }
}