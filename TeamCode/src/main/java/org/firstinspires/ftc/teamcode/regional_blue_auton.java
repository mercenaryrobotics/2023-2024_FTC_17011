package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="REGIONAL BlUE AUTON", group="Robot")
public class regional_blue_auton extends LinearOpMode {
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx backRightDrive = null;

    // Intake
    private Servo intakewrist = null;
    private CRServo intakeLeft = null;
    private CRServo intakeRight = null;

    // Arm
    private DcMotorEx pivot = null;
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

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Intake
        intakewrist = hardwareMap.get(Servo.class, "intakewrist");
        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");
        intakewrist.setPosition(0);

        // Pivot
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        pivot.setTargetPosition(0);
//        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        pivot.setPower(0.5);
    }

    int BackRightPos;
    int BackLeftPos;
    int FrontLeftPos;
    int FrontRightPos;

    private void drive(int FrontLeftTarget, int FrontRightTarget, int BackLeftTarget, int BackRightTarget, double speed) {
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BackRightPos -= BackRightTarget;
        BackLeftPos -= BackLeftTarget;
        FrontLeftPos -= FrontLeftTarget;
        FrontRightPos -= FrontRightTarget;

        backLeftDrive.setTargetPosition(BackLeftPos);
        backRightDrive.setTargetPosition(BackRightPos);
        frontLeftDrive.setTargetPosition(FrontLeftPos);
        frontRightDrive.setTargetPosition(FrontRightPos);

        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontRightDrive.setPower(speed);
        frontLeftDrive.setPower(speed);
        backRightDrive.setPower(speed);
        backLeftDrive.setPower(speed);

        while (opModeIsActive() && frontRightDrive.isBusy() && frontLeftDrive.isBusy() && backRightDrive.isBusy() && backLeftDrive.isBusy()) {
            idle();
        }
    }

    @Override
    public void runOpMode() {
        initializeMotors();

        pivot.setTargetPosition(0);
        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        waitForStart();

        drive(600, 600, 600, 600, 0.7);

        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        pivot.setTargetPosition(0);
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (true) {
            telemetry.addData("test", pivot.getCurrentPosition());
            telemetry.update();
        }
    }
}
