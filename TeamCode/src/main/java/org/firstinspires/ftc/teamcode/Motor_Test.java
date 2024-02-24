package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp (name = "TEST_MotorEncoders" ,group = "Test")

public class Motor_Test extends LinearOpMode {
    //Motor demo variables
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRigtDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx backRightDrive = null;

    public void initializeMotors()
    {
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "FL");
        frontRigtDrive = hardwareMap.get(DcMotorEx.class, "FR");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "BL");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "BR");

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRigtDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runOpMode()  {
        initializeMotors();

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("FL (0)", frontLeftDrive.getCurrentPosition());
            telemetry.addData("FR (1)", frontRigtDrive.getCurrentPosition());
            telemetry.addData("BL (2)", backLeftDrive.getCurrentPosition());
            telemetry.addData("BR (3)", backRightDrive.getCurrentPosition());
            updateTelemetry(telemetry);
        }
    }
}
