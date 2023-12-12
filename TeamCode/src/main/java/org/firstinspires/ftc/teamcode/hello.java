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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp (name = "Main2024CompCode" ,group = "Linear Opmode")

//@Disabled
public class hello extends LinearOpMode {
    private static final double NORMAL_SPEED = 0.95;
    private static final double SLOW_SPEED = 0.60;

    private IMU imu = null; // Control/Expansion Hub IMU

    //Motor demo variables
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx backRightDrive = null;

    private DcMotorEx climberMotorLeft = null;
    private DcMotorEx climberMotorRight = null;

    private DcMotorEx pivot = null;
    private DcMotorEx lift = null;


    private Servo climberHookLeft = null;
    private Servo climberHookRight = null;

    private Servo shooter = null;

    private Servo intakeWrist = null;
    private CRServo intakeLeft = null;
    private CRServo intakeRight = null;

    private Servo outtakeLeft = null;
    private Servo outtakeRight = null;

    private Servo outtakeWrist = null;
    private double intakeWristTarget;
    private IntakeWristState intakeWristState;

    enum IntakeWristState {
        DEFAULT,
        SLOW_MOVING,
        SLOW_DONE
    }



    private void initializeMotors()
    {
        // Drive
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "FL");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "FR");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "BL");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "BR");
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        lift = hardwareMap.get(DcMotorEx.class, "lift");

        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivot.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

lift.setPower(0.3);
        // Intake
        intakeLeft = hardwareMap.get(CRServo.class, "leftIntake");
        intakeRight = hardwareMap.get(CRServo.class, "rightIntake");
        intakeWrist = hardwareMap.get(Servo.class, "intakeWrist");
        outtakeWrist = hardwareMap.get(Servo.class, "outtakeWrist");



        intakeWrist.setDirection(Servo.Direction.FORWARD);
        moveIntakeWrist(0, IntakeWristState.DEFAULT);

         //Out-take
        outtakeLeft = hardwareMap.get(Servo.class, "outtakeLeft");
        outtakeRight = hardwareMap.get(Servo.class, "outtakeRight");
        outtakeRight.setDirection(Servo.Direction.REVERSE);

        // Climber
        climberMotorLeft = hardwareMap.get(DcMotorEx.class, "climberMotorLeft");
        climberMotorRight = hardwareMap.get(DcMotorEx.class, "climberMotorRight");
        climberHookLeft = hardwareMap.get(Servo.class, "climberHookLeft");
        climberHookRight = hardwareMap.get(Servo.class, "climberHookRight");


        climberHookLeft.setDirection(Servo.Direction.REVERSE);
        climberMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climberMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        climberMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climberMotorLeft.setTargetPosition(0);
        climberMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        climberMotorLeft.setPower(1);

        climberMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climberMotorRight.setTargetPosition(0);
        climberMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        climberMotorRight.setPower(1);

        pivot.setPower(1);
        sleep(1);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift .setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    private void initializeImu() {
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    private void moveIntakeWrist(double position, IntakeWristState mode) {
        intakeWristTarget = position;
        if (mode == IntakeWristState.DEFAULT) {
            intakeWrist.setPosition(intakeWristTarget);
        } else {
            intakeWristState = IntakeWristState.SLOW_MOVING;
        }
    }

    private void slowServoLoop() {
        if (intakeWristTarget + 0.01 >= intakeWrist.getPosition() && intakeWristTarget - 0.01 <= intakeWrist.getPosition()) {
            intakeWristState = IntakeWristState.SLOW_DONE;
        }
        else if (intakeWristState == IntakeWristState.SLOW_MOVING) {
            intakeWrist.setPosition(intakeWrist.getPosition() + ((intakeWristTarget - intakeWrist.getPosition()) / 35));
        }
    }

    private void joystickMecanumDrive() {

        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        double speed_multiplier;
        if (gamepad1.left_trigger > 0.5) {
            speed_multiplier = SLOW_SPEED;
        } else {
            speed_multiplier = NORMAL_SPEED;
        }


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = -(y + x + rx) / denominator; //A-RC has negative before parentheses and B-RC has no negative
        double backLeftPower = -(y - x + rx) / denominator;
        double frontRightPower = -(y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftDrive.setPower(frontLeftPower * speed_multiplier);
        backLeftDrive.setPower(backLeftPower * speed_multiplier);
        frontRightDrive.setPower(frontRightPower * speed_multiplier);
        backRightDrive.setPower(backRightPower * speed_multiplier);


    }

    private void fieldCentricDrive() {
        double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = -gamepad1.right_stick_x;

        // Calculate the current angle of the robot (yaw) relative to the field.
        double robotAngle = -Math.toRadians(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)); // Modify this to get the actual robot angle.

        // Calculate the field-centric components of movement.
        double fieldX = x * Math.cos(robotAngle) - y * Math.sin(robotAngle);
        double fieldY = x * Math.sin(robotAngle) + y * Math.cos(robotAngle);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(fieldY) + Math.abs(fieldX) + Math.abs(rx), 1);
        double frontLeftPower = (fieldY + fieldX + rx) / denominator;
        double backLeftPower = (fieldY - fieldX + rx) / denominator;
        double frontRightPower = (fieldY - fieldX - rx) / denominator;
        double backRightPower = (fieldY + fieldX - rx) / denominator;

        // Set the motor powers.
        frontLeftDrive.setPower(frontLeftPower);
        backLeftDrive.setPower(backLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backRightDrive.setPower(backRightPower);
    }

    private void climberFunctions() {
        /* only for the manual controls */
        if (gamepad1.dpad_up) {
            climberMotorLeft.setTargetPosition(climberMotorLeft.getTargetPosition() +- 20);
            climberMotorRight.setTargetPosition(climberMotorLeft.getTargetPosition() + 20);
        }
        else if (gamepad1.dpad_down){
            climberMotorLeft.setTargetPosition(climberMotorLeft.getTargetPosition() + 20);
            climberMotorRight.setTargetPosition(climberMotorLeft.getTargetPosition() - 20);
        }

        /* actual controls for comp climb */
        if (gamepad2.dpad_up){
            climberMotorLeft.setTargetPosition(-4000);
            climberMotorRight.setTargetPosition(4000);
        }
        else if (gamepad2.dpad_down){
            climberMotorLeft.setTargetPosition(0);
            climberMotorRight.setTargetPosition(0);
        }
        if (gamepad2.dpad_left) {
            climberHookLeft.setPosition(0);
            climberHookRight.setPosition(0);
        }
        else if (gamepad2.dpad_right){
            climberHookLeft.setPosition(0.4);
            climberHookRight.setPosition(0.42);
        }
        telemetry.addData("climberMotorLeft", climberMotorLeft.getCurrentPosition());
        telemetry.addData("climberMotorRight", climberMotorRight.getCurrentPosition());
        telemetry.update();

    }


    private void armFunctions() {
        /* for intake */
        if (gamepad1.left_bumper) {
            intakeLeft.setPower(1);
            intakeRight.setPower(1);
        } else if (gamepad1.right_bumper) {
            intakeLeft.setPower(-1);
            intakeRight.setPower(-1);
        }
        /* for intake wrist */
        else if (gamepad1.dpad_left) {
            moveIntakeWrist(0, IntakeWristState.SLOW_MOVING);
        }
        else if (gamepad1.dpad_right) {
            moveIntakeWrist(0.725, IntakeWristState.DEFAULT);
            intakeLeft.setPower(-1);
            intakeRight.setPower(-1);

        }
        else if (gamepad1.dpad_up) {
            moveIntakeWrist(0.05, IntakeWristState.SLOW_MOVING);


        } else {
            intakeLeft.setPower(0);
            intakeRight.setPower(0);

        /* outtake flaps */
            if (gamepad2.right_bumper) {
             outtakeLeft.setPosition(0.05);
             outtakeRight.setPosition(0.05);
            }
            else if (gamepad2.left_bumper) {
                outtakeLeft.setPosition(0.32);
                outtakeRight.setPosition(0.32);
            }
        /* outtake wrist */
            if (gamepad2.a) {
                lift.setTargetPosition(0);
            }
            else if (gamepad2.b) {
                lift.setTargetPosition(300);
            }

        /* lift */


        }
    }

    public void runOpMode()  {
        initializeMotors();
        initializeImu();
        waitForStart();
        imu.resetYaw();
        while (opModeIsActive()) {
            joystickMecanumDrive();
            fieldCentricDrive();

            //climberFunctions();
            armFunctions();
            slowServoLoop(); // for slow wrist control

            telemetry.addData("pivot", pivot.getCurrentPosition());
            telemetry.addData("lift", lift.getCurrentPosition());
            updateTelemetry(telemetry);
        }
    }
}