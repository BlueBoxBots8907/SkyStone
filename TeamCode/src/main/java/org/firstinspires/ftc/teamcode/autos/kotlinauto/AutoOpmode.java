package org.firstinspires.ftc.teamcode.autos.kotlinauto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.hardware.CollectionAndLiftMotors;
import org.firstinspires.ftc.teamcode.hardware.Servos;
import org.firstinspires.ftc.teamcode.hardware.Switches;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="Autonomous (Kotlin)", group = "drive")
public class AutoOpmode extends LinearOpMode {
    private CollectionAndLiftMotors cal;
    private Switches switches;
    private Servos hooks;
//
    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = 1f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos;//0 = col, 1 = row
    private static float[] leftPos;
    private static float[] rightPos;
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera webcam;

    private void deposit(int level) {
        cal.setLiftPowers(0.5, 0.5);
        idle();
//        ElapsedTime elapsedTime = new ElapsedTime();
//        elapsedTime.reset();
//        while(!isStopRequested() && elapsedTime.milliseconds() < level * 600 + 400) {
//            cal.setLiftPowers(0.252, 0.253);
//            idle();
//        }
//        if (level * 600 + 400 < 0) {
//            RobotLog.setGlobalErrorMsg("BAD " + level + ", " + level * 600 + ", " + level * 600 + 400);
//        }
        while(cal.getLiftHeight() <= 0.2 + level * 3.5 && !isStopRequested()) {
            idle();
        }
        cal.setLiftPowers(0.2, 0.2);
        sleep(200);
        hooks.setArmPosition(1);
        sleep(700);
        hooks.clawHalfOpen();
        sleep(300);
        cal.setLiftPowers(0.6, 0.6);
        sleep(300);
        hooks.clawClose();
        sleep(300);
        hooks.setArmPosition(0.29);
        cal.setLiftPowers(-0.25, -0.25);
        idle();
        sleep(level * 600 + 400);
        cal.setLiftPowers(0, 0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        midPos = new float[]{4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
        leftPos = new float[]{1f/8f+offsetX, 4f/8f+offsetY};
        rightPos = new float[]{6f/8f+offsetX, 4f/8f+offsetY};

        cal = new CollectionAndLiftMotors(hardwareMap);
        switches = new Switches(hardwareMap);
        hooks = new Servos(hardwareMap);
        boolean[] switchStates = switches.readSwitches();
        telemetry.addData("Switch Alliance", switchStates[0]);
        telemetry.addData("Switch Start Location", switchStates[1]);
        telemetry.addData("Switch Pull Foundation", switchStates[2]);
        telemetry.addData("Switch Parking Location", switchStates[3]);

        FieldUtil.Color color = switchStates[0] ? FieldUtil.Color.BLUE : FieldUtil.Color.RED;
        FieldUtil.Side side = switchStates[1] ? FieldUtil.Side.STONE : FieldUtil.Side.FOUNDATION;
        FieldUtil.Side parkingSide = switchStates[3] ? FieldUtil.Side.STONE : FieldUtil.Side.FOUNDATION;
        boolean doesPull = switchStates[2];

        if (color == FieldUtil  .Color.RED) {
            midPos = new float[] { 1f - midPos[0], midPos[1] };
            leftPos = new float[] { 1f - leftPos[0], leftPos[1] };
            rightPos = new float[] { 1f - rightPos[0], rightPos[1] };
        }

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();//open camera
        webcam.setPipeline(new AutoOpmode.StageSwitchingPipeline());//different stages
        webcam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        telemetry.addData("values", "left " + valLeft + "mid " + valMid + "right " + valRight);
        telemetry.update();

        AutoType autoType = new AutoType(
                color,
                side,
                parkingSide,
                valLeft <= valRight && valLeft <= valMid ? 0 : valMid <= valRight ? 1 : 2,
                doesPull);

        if (valLeft == 0) {
            autoType.skystoneIdx = 0;
        } else if (valMid == 0) {
            autoType.skystoneIdx = 1;
        } else {
            autoType.skystoneIdx = 2;
        }

        drive.getLocalizer().setPoseEstimate(FieldUtil.getStartingPose(autoType.color, autoType.side, autoType.doesPull));

        hooks.clawOpen();

//        sleep(1000);
//        hooks.clawClose();
//        sleep(2000);
//        deposit(1);

        if (autoType.side == FieldUtil.Side.STONE) {
            cal.setCollectionPowers(1, 1);
            hooks.setArmPosition(0.29);
            drive.followTrajectorySync(drive.trajectoryBuilder() // skystone 1
                    .splineTo(FieldUtil.getStonePose(autoType.color, autoType.skystoneIdx))
                    .forward(20).build());
            sleep(500);
            cal.setCollectionPowers(0, 0);
            hooks.clawClose();
            cal.setCollectionPowers(-0.5, -0.5);
            drive.followTrajectorySync(drive.trajectoryBuilder() // foundation pull + skystone 1 deposit
                    .reverse()
                    .splineTo(FieldUtil.getSkybridgePose(autoType.color, autoType.side))
                    .splineTo(FieldUtil.getFoundationPose(autoType.color)).build());
            hooks.foundationDown();
            deposit(0);
            drive.followTrajectorySync(drive.trajectoryBuilder() // foundation release
                    .splineTo(FieldUtil.getFoundationReleasePose(autoType.color)).build());
            hooks.foundationUp();
            cal.setCollectionPowers(0, 0);
            sleep(500);
            hooks.clawOpen();
            cal.setCollectionPowers(1, 1);
            if (autoType.skystoneIdx == 2) {
                drive.followTrajectorySync(drive.trajectoryBuilder()
                        .splineTo(FieldUtil.getSkybridgePose(autoType.color, autoType.side, true))
                        // .splineTo(Pose2d(-40.0, if (autoType.color == FieldUtil.Color.RED) -35.0 else 35.0, 180.0.toRadians))
                        .splineTo(FieldUtil.getStonePoseFive(autoType.color)).build());
            } else {
                drive.followTrajectorySync(drive.trajectoryBuilder() // skystone 2
                        .splineTo(FieldUtil.getSkybridgePose(autoType.color, autoType.side, true))
                        // .splineTo(Pose2d(-40.0, if (autoType.color == FieldUtil.Color.RED) -35.0 else 35.0, 180.0.toRadians))
                        .splineTo(FieldUtil.getStonePose(autoType.color, autoType.skystoneIdx + 3))
                        .forward(26).build());
            }
            sleep(500);
            hooks.clawClose();
            cal.setCollectionPowers(-0.5, -0.5);
            drive.followTrajectorySync(drive.trajectoryBuilder() // skystone 2 deposit
                    .reverse()
                    .splineTo(FieldUtil.getSkybridgePose(autoType.color, autoType.side))
                    .splineTo(FieldUtil.getDroppingPose(autoType.color)).build());
            cal.setCollectionPowers(0,0);
            deposit(1);
            drive.followTrajectorySync(drive.trajectoryBuilder() // navigating/parking
                    .splineTo(FieldUtil.getSkybridgePose(autoType.color, autoType.side)).build());
            cal.setCollectionPowers(0,0);
        } else if (autoType.side == FieldUtil.Side.FOUNDATION && autoType.doesPull) {
            drive.followTrajectorySync(drive.trajectoryBuilder() // foundation pull + skystone 1 deposit
                    .reverse()
                    .splineTo(FieldUtil.getFoundationPose(autoType.color)).build());
            hooks.foundationDown();
            sleep(500);
            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .splineTo(FieldUtil.getFoundationReleasePose(autoType.color)).build());
            drive.followTrajectorySync(drive.trajectoryBuilder() // foundation release
                    .back(15.0).build());
            hooks.foundationUp();
            sleep(500);
            drive.followTrajectorySync(drive.trajectoryBuilder() // navigating/parking
                    .splineTo(FieldUtil.getSkybridgePose(autoType.color, autoType.parkingSide)).build());
        } else {
            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .splineTo(FieldUtil.getSkybridgePose(autoType.color, autoType.parkingSide)).build());
        }
        hooks.clawOpen();
    }
    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private AutoOpmode.StageSwitchingPipeline.Stage stageToRenderToViewport = AutoOpmode.StageSwitchingPipeline.Stage.detection;
        private AutoOpmode.StageSwitchingPipeline.Stage[] stages = AutoOpmode.StageSwitchingPipeline.Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }

    }
}
