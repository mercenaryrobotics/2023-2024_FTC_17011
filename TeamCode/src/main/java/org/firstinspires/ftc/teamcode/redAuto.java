package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;
import org.opencv.core.MatOfPoint;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;


@Disabled
@Autonomous (name="Comp Red Auton", group="Robot")
public class redAuto extends LinearOpMode {
    public static double ORBITAL_FREQUENCY = 0.05;
    public static double SPIN_FREQUENCY = 0.25;
    public static double ORBITAL_RADIUS = 50;
    public static double SIDE_LENGTH = 10;

    public static double DRIVE_SPEED = 0.7;
    public static double TURN_SPEED = 0.3;

    static final double     COUNTS_PER_MOTOR_REV    = 537.7;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;
    static final double     WHEEL_DIAMETER_INCHES   = 3.779;
    static final double     COUNTS_PER_INCH         =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                    (WHEEL_DIAMETER_INCHES * 3.1415);

    //Motor demo variables
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx backRightDrive = null;

    private DcMotorEx climberMoter = null;

    //public static int isTeamRed = 1;
    //if -1, blue, if 1, red

    private int isStartBackdrop = 1;

    private int isStartStation = -1;
    private Servo intake = null;

    private DcMotorEx arm = null;

    private ElapsedTime runtime = new ElapsedTime();

    private FtcDashboard dashboard;
    int aprilTagValue = -1;
private void notStarted() {
    while (!isStarted()) {

        if (AutonIPCVariables.ballX < 250 && AutonIPCVariables.ballX > -50) {
            aprilTagValue = 5;
        } else if (AutonIPCVariables.ballX > -280 && AutonIPCVariables.ballX < -220) {
            aprilTagValue = 4;
        } else {
            aprilTagValue = 6;
        }
        telemetry.addData("x value", AutonIPCVariables.ballX);
        telemetry.addData("aprilTag value", aprilTagValue);
        //telemetry.addData("isTeamRed", isTeamRed);
        telemetry.update();

        //if (gamepad1.b) {
        //    isTeamRed = 1;
        //}

        //else if (gamepad1.x){
        //    isTeamRed = -1;
        //}

        //if (gamepad1.y) {
        //    int i = isStartBackdrop = 1;
        //}
        //else if (gamepad1.a) {
        //    int i = isStartStation = -1;
        //}
    }
}

private void station() {}
private void backDrop() {
    intake.scaleRange(0, 1);
    intake.setDirection(Servo.Direction.FORWARD);

    if (aprilTagValue==(4)){
        intake.setPosition(0.6);
        sleep(200);
        arm.setTargetPosition(-40);
        sleep(300);
        arm.setTargetPosition(-40);
        drive(1000,1000,1000,1000,0.6);
        sleep(200);
        drive(-700,700,-700,700,0.4);
        sleep(200);
        drive(300,300,300,300, 0.3);
        sleep(300);
        arm.setTargetPosition(0);
        sleep(100);
        intake.setPosition(0.3);
        arm.setTargetPosition(0);
        drive(-300,-300,-300,-300, 0.3);
        sleep(200);
        drive(700,-700,700,-700,0.4);
        sleep(150);
        drive(2050, -2050,2050,-2050, 0.5);
        sleep(200);
        drive(-490,490,490,-490,0.5);
        sleep(300);
        drive(1000,1000,1000,1000,0.25);
        sleep(300);
        intake.setPosition(0.6);
        sleep(300);
        arm.setTargetPosition(-50);
        sleep(500);
        drive(-1500,-1500,-1500,-1500,0.3);
        sleep(200);
        drive(-1000, 1000,-1000,1000, 0.6);
        sleep(300);
        drive(900, 900, 900, 900, 0.5);
        sleep(300);
        intake.setPosition(0.6);
        arm.setTargetPosition(-170);
        sleep(300);
        drive(400, 400, 400, 400, 0.2);
        sleep(500);
        intake.setPosition(0.3);
        sleep(500);
        drive(-300, -300, -300, -300, 0.2);
        arm.setTargetPosition(0);
        sleep(500);
        drive(800,-800,-800,800,0.5);
    }
    else if (aprilTagValue==(5)) {
        intake.setPosition(0.6);
        arm.setTargetPosition(-40);
        sleep(300);
        drive(1300,1300,1300,1300,0.4);
        sleep(300);
        arm.setTargetPosition(0);
        sleep(300);
        intake.setPosition(0.3);
        arm.setTargetPosition(0);
        sleep(200);
        drive(-700,-700,-700,-700,0.3);
        sleep(100);
        drive(2000, -2000,2000,-2000, 0.6);
        sleep(300);
        drive(-500,500,500,-500,0.5);
        sleep(300);
        drive(420,420,420,420,0.4);
        sleep(300);
        intake.setPosition(0.6);
        sleep(300);
        arm.setTargetPosition(-50);
        sleep(500);
        drive(-1100,-1100,-1100,-1100,0.3);
        sleep(200);
        drive(-970, 970,-970,970, 0.6);
        sleep(300);
        drive(900, 900, 900, 900, 0.5);
        sleep(300);
        intake.setPosition(0.6);
        arm.setTargetPosition(-150);
        sleep(300);
        drive(550, 550, 550, 550, 0.2);
        sleep(500);
        intake.setPosition(0.3);
        sleep(500);
        drive(-300, -300, -300, -300, 0.2);
        arm.setTargetPosition(0);
        sleep(500);
        drive(500,-500,-500,500,0.5);


    }
    else if (aprilTagValue==(6)){
        intake.setPosition(0.6);
        sleep(200);
        arm.setTargetPosition(-50);
        sleep(300);
        drive(1300,1300,1300,1300,0.6);
        sleep(200);
        drive(700,-700,700,-700,0.4);
        sleep(200);
        drive(200,200,200,200, 0.3);
        sleep(300);
        arm.setTargetPosition(0);
        sleep(200);
        intake.setPosition(0.3);
        arm.setTargetPosition(0);
        drive(-200,-200,-200,-200, 0.3);
        sleep(200);
        drive(-700,700,-700,700,0.4);
        sleep(150);
        drive(-500,-500,-500,-500,-0.5);
        sleep(200);
        drive(2050, -2050,2050,-2050, 0.5);
        sleep(200);
        drive(-550,550,550,-550,0.5);
        sleep(300);
        drive(300,300,300,300,0.25);
        sleep(300);
        intake.setPosition(0.6);
        sleep(500);
        drive(-900,-900,-900,-900,0.3);
        sleep(200);
        drive(-990, 990,-990,990, 0.6);
        sleep(300);
        arm.setTargetPosition(-190);
        sleep(300);
        drive(800, 800, 800, 800, 0.5);
        sleep(300);
        intake.setPosition(0.6);
        arm.setTargetPosition(-190);
        sleep(300);
        drive(550, 550, 550, 550, 0.2);
        sleep(500);
        intake.setPosition(0.3);
        sleep(500);
        drive(-300, -300, -300, -300, 0.2);
        arm.setTargetPosition(0);
        sleep(500);
        drive(800,-800,-800,800,0.5);
    }

    while (opModeIsActive()) {

        telemetry.addData("x value", AutonIPCVariables.ballX);
        telemetry.addData("aprilTag value", aprilTagValue);
        telemetry.update();
    }
    }

    int BackRightPos;
    int BackLeftPos;
    int FrontLeftPos;
    int FrontRightPos;
    OpenCvWebcam webcam;

    private void drive(int FrontLeftTarget, int FrontRightTarget, int BackLeftTarget, int BackRightTarget, double speed) {
        BackRightPos += BackRightTarget;
        BackLeftPos -= BackLeftTarget;
        FrontLeftPos -= FrontLeftTarget;
        FrontRightPos += FrontRightTarget;
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
    private void initializeMotors() {
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "FL");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "FR");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "BL");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "BR");
        //climberMoter = hardwareMap.get(DcMotorEx.class, "climber");
        arm = hardwareMap.get(DcMotorEx.class, "Arm");
        intake = hardwareMap.get(Servo.class, "intake");


        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        //climberMoter.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //climberMoter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //climberMoter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setPower(0.2);
        arm.setTargetPositionTolerance(1);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }

    private void initializeDashboard() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    private void initializeVision() {
        /* Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        webcam.setPipeline(new RedOurOpenCVPipeline());
        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {

                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addLine(String.format(Locale.ENGLISH, "ERROR: Camera Init %d", errorCode));
            }
        });

    }

    public void autoStuff(int a){
        if (a == 1){

        }
        else if (a == 2){

        }
        else {

        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initializeMotors();
        initializeDashboard();
        initializeVision();
        FtcDashboard.getInstance().startCameraStream(webcam, 60);
        notStarted();

        if (isStartBackdrop == 1)
            backDrop();

        if (isStartStation == -1)
            station();

    }
}


class RedOurOpenCVPipeline extends OpenCvPipeline {

    /*
     * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
     * highly recommended to declare them here as instance variables and re-use them for
     * each invocation of processFrame(), rather than declaring them as new local variables
     * each time through processFrame(). This removes the danger of causing a memory leak
     * by forgetting to call mat.release(), and it also reduces memory pressure by not
     * constantly allocating and freeing large chunks of memory.
     */

    /**
     * Get largest contour index in list of OpenCV contours
     * @param contours contours to search
     * @return contour index
     */
    private int getLargestContourIndex(List<MatOfPoint> contours) {
        // Now search the list for the new largest "blob"
        double largest_area = 0.0;
        int largest_contour_index = -1;

        for (int i = 0; i < contours.size(); i++) {// iterate through each contour.
            double area = Imgproc.contourArea(contours.get(i),false); // Find the area of contour
            if ((area > largest_area) && (area > RedAutonConstants.minBallArea)) {
                largest_area=area;
                largest_contour_index=i; // Store the index of largest contour
            }
        }

        return largest_contour_index;
    }

    /**
     * Get largest contour area in list of OpenCV contours
     * @param contours contours to search
     * @return contour area
     */
    private double getLargestContourArea(List<MatOfPoint> contours) {
        // Now search the list for the new largest "blob"
        double largest_area = 0.0;

        for (int i = 0; i < contours.size(); i++) {// iterate through each contour.
            double area = Imgproc.contourArea(contours.get(i),false); // Find the area of contour
            if ((area > largest_area) && (area > RedAutonConstants.minBallArea)) {
                largest_area=area;
            }
        }

        return largest_area;
    }

    /**
     * Get centers of contour "blobs"
     * @param contours contours to search
     * @return contour centers
     */
    private Point[] getContourCenters(List<MatOfPoint> contours) {
        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
        Point[] centers = new Point[contours.size()];

        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            centers[i] = new Point();
            Imgproc.minEnclosingCircle(contoursPoly[i], centers[i], null);
        }

        return centers;
    }

    /**
     * Get radii of contour "blobs"
     * @param contours contours to search
     * @return contour radii
     */
    private float[][] getContourRadii(List<MatOfPoint> contours) {
        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
        float[][] radii = new float[contours.size()][1];

        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            Imgproc.minEnclosingCircle(contoursPoly[i], null, radii[i]);
        }

        return radii;
    }

    /**
     * Get contour polygons
     * @param contours contours to search
     * @return contour polys
     */
    private List<MatOfPoint> getContourPolys(List<MatOfPoint> contours) {
        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];

        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            Imgproc.minEnclosingCircle(contoursPoly[i], null, null);
        }
        List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
        for (MatOfPoint2f poly : contoursPoly) {
            contoursPolyList.add(new MatOfPoint(poly.toArray()));
        }

        return contoursPolyList;
    }

    @Override
    public Mat processFrame(Mat input) {
        /*
         * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
         * will only dereference to the same image for the duration of this particular
         * invocation of this method. That is, if for some reason you'd like to save a copy
         * of this particular frame for later use, you will need to either clone it or copy
         * it to another Mat.
         */
        Mat DisplayImage = input.clone();
        Mat TestImage = input.clone();

        //Convert to HSV for better color range definition
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);

        //Filter pixels outside the desired color range

            Scalar color_lower = new Scalar(RedAutonConstants.color_lower_h, RedAutonConstants.color_lower_s, RedAutonConstants.color_lower_v);
            Scalar color_upper = new Scalar(RedAutonConstants.color_upper_h, RedAutonConstants.color_upper_s, RedAutonConstants.color_upper_v);
            Core.inRange(input, color_lower, color_upper, input);


        //Core.inRange(input, color_lower, color_upper, TestImage);


        //De-speckle the image
        Mat element = Imgproc.getStructuringElement(RedAutonConstants.elementType,
                new Size(2 * RedAutonConstants.kernelSize + 1,
                        2 * RedAutonConstants.kernelSize + 1),
                new Point(RedAutonConstants.kernelSize,
                        RedAutonConstants.kernelSize));
        Imgproc.erode(input, input, element);
        Imgproc.erode(input, TestImage, element);

        // Find blobs in the image. Actually finds a list of contours which will need to be processed later
        List<MatOfPoint> contours = new ArrayList<>();
        final Mat hierarchy = new Mat();
        Imgproc.findContours(input, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Filter out blobs larger than maxBallArea
        contours.removeIf(c -> (Imgproc.contourArea(c) > RedAutonConstants.maxBallArea));

        // Get largest contour (hopefully ball)
        int largest_contour_index = getLargestContourIndex(contours);
        AutonIPCVariables.foundBallArea = getLargestContourArea(contours);
        boolean foundBall = largest_contour_index > -1;

        // Find the contours and bounding regions
        Point[] centers = getContourCenters(contours);
        float[][] radii = getContourRadii(contours);
        List<MatOfPoint> contoursPolyList = getContourPolys(contours);

        // Draw shape and/or contour on original image
        Scalar color;
        for (int i = 0; i < contours.size(); i++) {
            if (i == largest_contour_index) {
                color = new Scalar(0, 255, 0);
            } else {
                color = new Scalar(255, 0, 0);
            }

            if (RedAutonConstants.drawContours) {
                Imgproc.drawContours(DisplayImage, contoursPolyList, i, color);
            }
            if (RedAutonConstants.drawCircle) {
                Imgproc.circle(DisplayImage, centers[i], (int) radii[i][0], color, 2);
            }
        }

        // IPC
        AutonIPCVariables.ballExists = foundBall;

        if (foundBall) {
            AutonIPCVariables.ballX = centers[largest_contour_index].x - (input.width() / 2.0);
            AutonIPCVariables.ballY = centers[largest_contour_index].y - (input.height() / 2.0);
        }

        /*
         * NOTE: to see how to get data from your pipeline to your OpMode as well as how
         * to change which stage of the pipeline is rendered to the viewport when it is
         * tapped, please see {@link PipelineStageSwitchingExample}
         */
        DisplayImage.copyTo(input);
        DisplayImage.release();

        TestImage.copyTo((input));
        TestImage.release();
        return input;
    }

}