package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "NewCompCode" ,group = "Linear Opmode")

//@Disabled
public class hello2 extends LinearOpMode {
    private static final double NORMAL_SPEED = 1;
    private static final double SLOW_SPEED = 0.40;

    private IMU imu = null; // Control/Expansion Hub IMU

    //Motor demo variables
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backLeftDrive = null;
    private Servo intakewrist = null;
    private CRServo intakeLeft = null;
//    private DcMotorEx pivot = null;
//    private DcMotorEx lift = null;
    private CRServo intakeRight = null;
//    private Servo outtakeLeft = null;
//    private Servo outtakeRight = null;
//
//    private Servo outtakeWrist = null;
//    private double outtakeWristTarget;

    private DcMotorEx climberMotorLeft = null;
    private DcMotorEx climberMotorRight = null;
    private Servo climberHookLeft = null;
    private Servo climberHookRight = null;
    private SlowServoState intakeWristState = SlowServoState.SLOW_DONE;
    private SlowServoState outtakeWristState = SlowServoState.SLOW_DONE;
    private double intakeWristTarget;

    private DcMotorEx backRightDrive = null;

    enum SlowServoState {
        DEFAULT,
        SLOW_MOVING,
        SLOW_DONE
    }

    enum robotState {
        Idle,
        PickUpInitial,
        PickUpTransfer,
        ScoreL1,
        ScoreL2,
        ScoreL3,
        ScoreL4,
        ScoreOuttake
    }

    public robotState currentState = robotState.Idle;

    private void initializeMotors() {
        // Drive
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "FL");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "FR");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "BL");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "BR");

        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakewrist = hardwareMap.get(Servo.class, "intakewrist");
//        outtakeWrist = hardwareMap.get(Servo.class, "outtakeWrist");
//        outtakeLeft = hardwareMap.get(Servo.class, "outtakeLeft");
//        outtakeRight = hardwareMap.get(Servo.class, "outtakeRight");
        moveIntakeWrist(0.03, SlowServoState.DEFAULT);
        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        climberMotorLeft = hardwareMap.get(DcMotorEx.class, "climberMotorLeft");
        climberMotorRight = hardwareMap.get(DcMotorEx.class, "climberMotorRight");

        climberHookLeft = hardwareMap.get(Servo.class, "climberHookLeft");
        climberHookRight = hardwareMap.get(Servo.class, "climberHookRight");
        climberHookRight.setDirection(Servo.Direction.REVERSE);

        climberMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climberMotorLeft.setTargetPosition(0);
        climberMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        climberMotorLeft.setPower(0.2);

        climberMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climberMotorRight.setTargetPosition(0);
        climberMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        climberMotorRight.setPower(0.7);

        climberMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climberMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Lift
//        lift = hardwareMap.get(DcMotorEx.class, "lift");
//        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        lift.setPower(0);


    }

    private void initializeImu() {
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

//    private void armFunctions() {
//
//        if (gamepad2.right_bumper) {
//            currentState = robotState.PickUpTransfer;
//        } else if (gamepad2.left_bumper) {
//            currentState = robotState.PickUpInitial;
//        } else if (gamepad2.a) {
//            if (currentState == robotState.ScoreL1) {
//                currentState = robotState.ScoreOuttake;
//            } else {
//                currentState = robotState.ScoreL1;
//                sleep(500);
//            }
//        } else if (gamepad2.b) {
//            if (currentState == robotState.ScoreL2) {
//                currentState = robotState.ScoreOuttake;
//            } else {
//                currentState = robotState.ScoreL2;
//                sleep(500);
//            }
//        } else if (gamepad2.x) {
//            currentState = robotState.ScoreL3;
//        } else if (gamepad2.y) {
//            currentState = robotState.ScoreL4;
//        } else if (gamepad2.back) {
//            currentState = robotState.Idle;
//            moveIntakeWrist(0.05, SlowServoState.SLOW_MOVING);
//            outtakeRight.setPosition(0.32);
//            outtakeLeft.setPosition(0.32);
//            lift.setTargetPosition(0);
//            pivot.setTargetPosition(0);
//            moveOuttakeWrist(0, SlowServoState.SLOW_MOVING);
//        }
//        switch (currentState) {
//            case Idle:
//                // Not implemented
//                break;
//            case PickUpInitial:
//                moveIntakeWrist(0.03, SlowServoState.SLOW_MOVING);
//                if (gamepad1.left_bumper) {
//                    intakeLeft.setPower(1);
//                    intakeRight.setPower(1);
//                } else {
//                    intakeLeft.setPower(0);
//                    intakeRight.setPower(0);
//                }
//                break;
//            case PickUpTransfer:
//                moveIntakeWrist(0.7, SlowServoState.DEFAULT);
//                if (gamepad1.right_bumper) {
//                    intakeLeft.setPower(-1);
//                    intakeRight.setPower(-1);
//                } else {
//                    intakeLeft.setPower(0);
//                    intakeRight.setPower(0);
//                }
//                break;
//            case ScoreL1:
//                lift.setTargetPosition(719);
//                pivot.setTargetPosition(-45);
//                moveOuttakeWrist(0.4, SlowServoState.DEFAULT);
//                break;
//            case ScoreL2:
//                lift.setTargetPosition(719);
//                pivot.setTargetPosition(-106);
//                moveOuttakeWrist(0.44, SlowServoState.DEFAULT);
//                // moveOuttakeWrist(gamepad2.right_trigger, SlowServoState.DEFAULT);
//                break;
//            case ScoreL3:
//                lift.setTargetPosition(729);
//                pivot.setTargetPosition(-106);
//                moveOuttakeWrist(0.44, SlowServoState.DEFAULT);
//                // moveOuttakeWrist(gamepad2.right_trigger, SlowServoState.DEFAULT);
//                break;
//            case ScoreL4:
//                break;
//            case ScoreOuttake:
//                outtakeRight.setPosition(0.05);
//                outtakeLeft.setPosition(0.05);
//                break;
//            }
//
//            if (gamepad1.right_bumper) {
//                currentState = robotState.PickUpTransfer;
//            } else if (gamepad1.left_bumper) {
//                currentState = robotState.PickUpInitial;
//            }
//        }
    private void moveIntakeWrist(double position, SlowServoState mode) {
        intakeWristTarget = position;
        if (mode == SlowServoState.DEFAULT) {
            intakewrist.setPosition(intakeWristTarget);
        } else {
            intakeWristState = SlowServoState.SLOW_MOVING;
        }
    }
//    private void moveOuttakeWrist(double position, SlowServoState mode) {
//        outtakeWristTarget = position;
//        if (mode == SlowServoState.DEFAULT) {
//            outtakeWrist.setPosition(outtakeWristTarget);
//        } else {
//            outtakeWristState = SlowServoState.SLOW_MOVING;
//        }
//    }
    private void slowServoLoop() {
        telemetry.addData("test", intakeWristState.toString());
        if (intakeWristTarget + 0.01 >= intakewrist.getPosition() && intakeWristTarget - 0.01 <= intakewrist.getPosition()) {
            intakeWristState = SlowServoState.SLOW_DONE;
        }
        else if (intakeWristState == SlowServoState.SLOW_MOVING) {
            intakewrist.setPosition(intakewrist.getPosition() + ((intakeWristTarget - intakewrist.getPosition()) / 200));
            telemetry.addData("uhhhh", intakewrist.getPosition() + ((intakeWristTarget - intakewrist.getPosition()) / 200));
        }
//
//        if (outtakeWristTarget + 0.01 >= outtakeWrist.getPosition() && outtakeWristTarget - 0.01 <= outtakeWrist.getPosition()) {
//            outtakeWristState = SlowServoState.SLOW_DONE;
//        }
//        else if (outtakeWristState == SlowServoState.SLOW_MOVING) {
//            outtakeWrist.setPosition(outtakeWrist.getPosition() + ((outtakeWristTarget - outtakeWrist.getPosition()) / 45));
//        }
   }
    private void climberFunctions() {
        /* actual controls for comp climb */
        if (gamepad1.dpad_up) {
            climberMotorLeft.setTargetPosition(3000);
            climberMotorRight.setTargetPosition(3000);
        }
        else if (gamepad1.dpad_down) {
            climberMotorLeft.setTargetPosition(0);
            climberMotorRight.setTargetPosition(0);
        }
        if (gamepad1.dpad_left) {
            climberHookLeft.setPosition(0.02);
            climberHookRight.setPosition(0.055);
        } else if (gamepad1.dpad_right) {
            climberHookLeft.setPosition(0.5);
            climberHookRight.setPosition(0.55);
        }
//        if (gamepad2.dpad_up) {
//            climberMotorLeft.setTargetPosition(climberMotorLeft.getTargetPosition() + 1);
//        } else if (gamepad2.dpad_down) {
//            climberMotorLeft.setTargetPosition(climberMotorLeft.getTargetPosition() - 1);
//        }
//
//        if (gamepad2.dpad_left) {
//            climberMotorRight.setTargetPosition(climberMotorRight.getTargetPosition() + 1);
//        } else if (gamepad2.dpad_right) {
//            climberMotorRight.setTargetPosition(climberMotorRight.getTargetPosition() - 1);
//        }
        telemetry.addData("climberMotorLeft", climberMotorLeft.getCurrentPosition());
        telemetry.addData("climberMotorRight", climberMotorRight.getCurrentPosition());
        }

    private void joystickMecanumDrive() {

        double y = 0;
        double x = 0;
        double rx = 0;

        if (!gamepad1.right_bumper) {
             y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
             x = gamepad1.left_stick_x;
             rx = gamepad1.right_stick_x;
        }

        double speed_multiplier;
        if (gamepad1.left_trigger > 0.5) {
            speed_multiplier = SLOW_SPEED;
        } else {
            speed_multiplier = NORMAL_SPEED;
        }


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(rx) + Math.abs(x), 1);
        double frontLeftPower = -(y + x + rx) / denominator; //A-RC has negative before parentheses and B-RC has no negative
        double backLeftPower = -(y - x + rx) / denominator;
        double frontRightPower = -(y - x - rx) / denominator;
        double backRightPower = -(y + x - rx) / denominator;

        frontLeftDrive.setPower(frontLeftPower * speed_multiplier);
        backLeftDrive.setPower(backLeftPower * speed_multiplier);
        frontRightDrive.setPower(frontRightPower * speed_multiplier);
        backRightDrive.setPower(backRightPower * speed_multiplier);

        telemetry.addData("backRightPower", backRightPower);
        telemetry.addData("backLeftPower", backLeftPower);
        telemetry.addData("frontRightPower", frontRightPower);
        telemetry.addData("frontLeftPower", frontLeftPower);

    }

    public void runOpMode()  {
        initializeMotors();
        initializeImu();
        waitForStart();
        imu.resetYaw();
        while (opModeIsActive()) {
            joystickMecanumDrive();
            //fieldCentricDrive();
//            armFunctions();
            climberFunctions();
            slowServoLoop();
            updateTelemetry(telemetry);
        }
    }
}