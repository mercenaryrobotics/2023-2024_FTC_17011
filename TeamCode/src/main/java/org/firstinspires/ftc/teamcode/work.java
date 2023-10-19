package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "pls work" ,group = "Linear Opmode")

//@Disabled
public class work extends LinearOpMode {
    //Dashboard demo variables
    public static double ORBITAL_FREQUENCY = 0.05;
    public static double SPIN_FREQUENCY = 0.25;
    public static double ORBITAL_RADIUS = 50;
    public static double SIDE_LENGTH = 10;

    public static double INTAKE_OPEN_POS = 0.0;
    public static double INTAKE_CLOSE_POS = 0.4;

    public static int ARM_DRIVE = 30;
    public static int ARM_SCORE = 740;
    public static int ARM_PICKUP = 110;
    public static double ARM_POWER = 0.35;

    //Motor demo variables
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx backRightDrive = null;

    private DcMotorEx climberMotor = null;

    private DcMotorEx wrist = null;

    private Servo intake = null;

    private DcMotorEx arm = null;

    public void initializeMotors()
    {
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "FL");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "FR");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "BL");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "BR");
        //climberMotor = hardwareMap.get(DcMotorEx.class, "climber");
        arm = hardwareMap.get(DcMotorEx.class, "Arm");
        wrist = hardwareMap.get(DcMotorEx.class, "wrist");
        intake = hardwareMap.get(Servo.class, "intake");


        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //climberMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //climberMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //climberMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setPower(ARM_POWER);
        arm.setTargetPositionTolerance(2);
        arm.setTargetPosition(ARM_DRIVE);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }

    private static void rotatePoints(double[] xPoints, double[] yPoints, double angle) {
        for (int i = 0; i < xPoints.length; i++) {
            double x = xPoints[i];
            double y = yPoints[i];
            xPoints[i] = x * Math.cos(angle) - y * Math.sin(angle);
            yPoints[i] = x * Math.sin(angle) + y * Math.cos(angle);
        }
    }


    public void dashboardDemo(){
        double time = getRuntime();

        double bx = ORBITAL_RADIUS * Math.cos(2 * Math.PI * ORBITAL_FREQUENCY * time);
        double by = ORBITAL_RADIUS * Math.sin(2 * Math.PI * ORBITAL_FREQUENCY * time);
        double l = SIDE_LENGTH / 2;

        double[] bxPoints = { l, -l, -l, l };
        double[] byPoints = { l, l, -l, -l };
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
        double frontRightPower = -(y - x - rx) / denominator;
        double backRightPower = -(y + x - rx) / denominator;

        frontLeftDrive.setPower(frontLeftPower);
        backLeftDrive.setPower(backLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backRightDrive.setPower(backRightPower);


    }

    private void climber() {
        if (gamepad2.y){
            climberMotor.setPower(1);
        }
        else if (gamepad2.a) {
            climberMotor.setPower(-1);
        }
        else {
            climberMotor.setPower(0);
        }

    }

    public void armFunctions() {
        if (gamepad1.right_bumper){
            if (intake.getPosition() == INTAKE_OPEN_POS) {
                intake.setPosition(INTAKE_CLOSE_POS);
            } else {
                intake.setPosition(INTAKE_OPEN_POS);
            }
        }

        if (gamepad2.a) {
            arm.setTargetPosition(ARM_DRIVE);
        } else if (gamepad2.b) {
            arm.setTargetPosition(ARM_PICKUP);
        }
        else if (gamepad2.x) {
            arm.setTargetPosition(ARM_SCORE);
        }
    }




    public void runOpMode()  {
        initializeMotors();
        intake.scaleRange(0,1);
        intake.setDirection(Servo.Direction.FORWARD);
        intake.setPosition(0);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("arm pos", arm.getCurrentPosition());
            telemetry.addData("wrist pos", wrist.getCurrentPosition());


//            telemetry.addData("climber pos", climberMotor.getCurrentPosition());
            updateTelemetry(telemetry);

            //joystickTankDrive();
            joystickMecanumDrive();
            armFunctions();
        }
    }
}
