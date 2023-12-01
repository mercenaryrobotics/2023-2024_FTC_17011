package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp (name = "Main2024CompCode" ,group = "Linear Opmode")

//@Disabled
public class hello extends LinearOpMode {
    //Dashboard demo variables
    public static double ORBITAL_FREQUENCY = 0.05;
    public static double SPIN_FREQUENCY = 0.25;
    public static double ORBITAL_RADIUS = 50;
    public static double SIDE_LENGTH = 10;
    public static double SPEED_MULTIPLIER = 1.00;


    public static int ARM_DRIVE = 0;

    private IMU imu = null;      // Control/Expansion Hub IMU

    //Motor demo variables
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx backRightDrive = null;

    //private DcMotorEx climberMotor = null;

    private DcMotorEx climberLeft = null;

    private Servo climberLeftArm = null;

    private Servo shooter = null;

    private CRServo intake = null;

    private Servo axe = null;

    private DcMotorEx pivot = null;
    private Servo wrist = null;

    public void initializeMotors() {
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "FL");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "FR");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "BL");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "BR");

        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        intake = hardwareMap.get(CRServo.class, "intake");
        //shooter = hardwareMap.get(Servo.class, "shooter");

        climberLeft = hardwareMap.get(DcMotorEx.class, "climberleft");

        climberLeftArm = hardwareMap.get(Servo.class, "climberleftarm");

        axe = hardwareMap.get(Servo.class, "axe");
        wrist = hardwareMap.get(Servo.class, "wrist");


        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        climberLeft.setTargetPosition(0);
        climberLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        climberLeft.setPower(0.25);

        pivot.setTargetPosition(0);
        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivot.setPower(0.25);

        wrist.setPosition(0);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    private static void rotatePoints(double[] xPoints, double[] yPoints, double angle) {
        for (int i = 0; i < xPoints.length; i++) {
            double x = xPoints[i];
            double y = yPoints[i];
            xPoints[i] = x * Math.cos(angle) - y * Math.sin(angle);
            yPoints[i] = x * Math.sin(angle) + y * Math.cos(angle);
        }
    }


    public void dashboardDemo() {
        double time = getRuntime();

        double bx = ORBITAL_RADIUS * Math.cos(2 * Math.PI * ORBITAL_FREQUENCY * time);
        double by = ORBITAL_RADIUS * Math.sin(2 * Math.PI * ORBITAL_FREQUENCY * time);
        double l = SIDE_LENGTH / 2;

        double[] bxPoints = {l, -l, -l, l};
        double[] byPoints = {l, l, -l, -l};
        rotatePoints(bxPoints, byPoints, 2 * Math.PI * SPIN_FREQUENCY * time);
        for (int i = 0; i < 4; i++) {
            bxPoints[i] += bx;
            byPoints[i] += by;
        }


        sleep(20);
    }

    private void joystickMecanumDrive() {

        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = -(y + x + rx) / denominator; //A-RC has negative before parentheses and B-RC has no negative
        double backLeftPower = -(y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftDrive.setPower(frontLeftPower * SPEED_MULTIPLIER);
        backLeftDrive.setPower(backLeftPower * SPEED_MULTIPLIER);
        frontRightDrive.setPower(frontRightPower * SPEED_MULTIPLIER);
        backRightDrive.setPower(backRightPower * SPEED_MULTIPLIER);


    }

    public void armFunctions() {
        if (gamepad1.right_bumper) {
            intake.setPower(1);
        } else if (gamepad1.left_bumper) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }

        if (gamepad1.x) {
            axe.setPosition(0);
        }
        else if (gamepad1.y){
            axe.setPosition(0.53);
        }
        else if (gamepad1.a) {
            axe.setPosition(0.35);
        }
        else if (gamepad1.b)      {
            axe.setPosition(0.17);
        }

        /*int encoderCount = arm.getCurrentPosition();
        if (gamepad2.a) {
            arm.setTargetPosition(ARM_DRIVE);
        } else if (gamepad2.right_trigger > 0.3) {
            arm.setTargetPosition(encoderCount + Math.round(40 * gamepad2.right_trigger));
        } else if (gamepad2.left_trigger > 0.3) {
            arm.setTargetPosition(encoderCount - Math.round(40 * gamepad2.left_trigger));
        }*//*else if (gamepad2.b) {
            arm.setTargetPosition(ARM_FRONTSCORE);
        }
        else if (gamepad2.x) {
            arm.setTargetPosition(ARM_SCORE);
        }*/

    }

    private void fieldCentricDrive() {
        double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = -gamepad1.right_stick_x;
        double lf = -gamepad2.left_stick_y;

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


    public void runOpMode() {
        initializeMotors();
        //intake.scaleRange(0,1);
        //intake.setDirection(Servo.Direction.FORWARD);
        //intake.setPosition(0.5);

        //shooter.scaleRange(0,1);
        imu.resetYaw();
        telemetry.addLine("test2");
        updateTelemetry(telemetry);
        waitForStart();
        telemetry.addLine(String.valueOf(opModeIsActive()));
        updateTelemetry(telemetry);
        while (opModeIsActive()) {

//            telemetry.addData("climber pos", climberMotor.getCurrentPosition());
            telemetry.addLine("test");
            updateTelemetry(telemetry);

            //joystickTankDrive();
            joystickMecanumDrive();
            armFunctions();
            //fieldCentricDrive();
        }
    }
}