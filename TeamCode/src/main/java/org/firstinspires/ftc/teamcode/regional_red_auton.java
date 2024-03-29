package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@Autonomous(name="REGIONAL RED AUTON", group="A")
public class regional_red_auton extends LinearOpMode {
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx backRightDrive = null;

    // Intake
    private Servo intakewrist = null;
    private CRServo intakeLeft = null;
    int aprilTagValue = -1;
    private CRServo intakeRight = null;
    private ElapsedTime runtime = new ElapsedTime();

    private FtcDashboard dashboard;
    OpenCvWebcam webcam;
    // Arm
    private DcMotorEx pivot = null;

    enum AutoState {
        PRE_INIT,
        DOING_STATION,
        POST,
        DONE
    }

    AutoState currentState = AutoState.PRE_INIT;

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

            telemetry.addData("x value", AutonIPCVariables.ballX);
            telemetry.addData("aprilTag value", aprilTagValue);

            idle();
        }
    }
    private void initializeDashboard() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }
    private void notStarted() {
        while (!isStarted()) {

            if (AutonIPCVariables.ballX > -134 && AutonIPCVariables.ballX < 100) {
                aprilTagValue = 2;
            } else if (AutonIPCVariables.ballX < 290 && AutonIPCVariables.ballX > 270) {
                aprilTagValue = 3;
            } else {
                aprilTagValue = 1;
            }
            telemetry.addData("x value", AutonIPCVariables.ballX);
            telemetry.addData("aprilTag value", aprilTagValue);
            telemetry.update();


        }
    }
    private void station() {

        intakewrist.setPosition(0.05);
        drive(650,650,650,650,0.7);
        sleep(500);
        intakeLeft.setPower(-1);
        intakeRight.setPower(-1);

//        if (aprilTagValue==(3)){
//
//        }
//        else if (aprilTagValue==(2)) {
//
//
//        }
//        else if (aprilTagValue==(1)){
//
//        }
    }
    @Override
    public void runOpMode() {
        initializeMotors();
        initializeDashboard();
        initializeVision();
        FtcDashboard.getInstance().startCameraStream(webcam, 60);

        while (!opModeIsActive()) {
            notStarted();
        } while (opModeIsActive()) {
            switch (currentState) {
                case PRE_INIT:
                    pivot.setTargetPosition(0);
                    pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    waitForStart();
//                    drive(500,500,500,500,0.7);
                    currentState = AutoState.DOING_STATION;
                    break;
                case DOING_STATION:
                    station();
                    currentState = AutoState.POST;
                    break;
                case POST:
                    pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    pivot.setTargetPosition(0);
                    pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    currentState = AutoState.DONE;
                    break;
                case DONE:
                    // Auto Over
                    break;
            }
            if (isStopRequested()) {
                break;
            }
        }
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
        webcam.setPipeline(new RedRegionalPipeline());
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
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                // webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addLine(String.format(Locale.ENGLISH, "ERROR: Camera Init %d", errorCode));
            }
        });

    }

}


class RedRegionalPipeline extends OpenCvPipeline {

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

        TestImage.copyTo((input));
        TestImage.release();
        return DisplayImage;
    }

}