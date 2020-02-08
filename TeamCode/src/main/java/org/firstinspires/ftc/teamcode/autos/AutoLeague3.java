package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
import org.openftc.easyopencv.OpenCvInternalCamera;

import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.revextensions2.ExpansionHubServo;

import java.util.ArrayList;
import java.util.List;




/**
 * Created by maryjaneb  on 11/13/2016.
 *
 * nervere
 *st ticks
 *  * 60 1680
 *  * 40 1120
 *  * 20 560
 * monitor: 640 x 480
 *YES
 */
@Autonomous(name= "Autonomous(OpenCV)", group="Sky autonomous")
//@Disabled//comment out this line before using
public class  AutoLeague3 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = -1f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera webcam;




    @Override
    public void runOpMode() throws InterruptedException {

        boolean[] switchStates;

        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        Servos hooks = new Servos(hardwareMap);
        Sampling detection = new Sampling();
        Switches switches = new Switches(hardwareMap);
        CollectionAndLiftMotors cal = new CollectionAndLiftMotors(hardwareMap);

        switchStates = switches.readSwitches();
        switchStates[2] = false;
        hooks.initPower();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();//open camera
        webcam.setPipeline(new StageSwitchingPipeline());//different stages
        webcam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            if (switchStates[1]) {
                telemetry.addData("Values", valLeft + "   " + valMid + "   " + valRight);
                telemetry.addData("Height", rows);
                telemetry.addData("Width", cols);

                telemetry.update();
                sleep(100);
                //call movement functions
                if (switchStates[0]) {
                    if (valLeft == 0) {
                        hooks.setArmPosition(0.94);
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .splineTo(new Pose2d(27, -4, Math.toRadians(-30)))
                                        .build()
                        );
                        cal.setCollectionPowers(0.5, 0.5);
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .forward(10)
                                        .build()
                        );
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .forward(9)
                                        .build()
                        );
                        sleep(500);
                        hooks.setArmPosition(1);
                        hooks.clawClose();
                        sleep(750);
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .back(17)
                                        .build()
                        );
                        cal.setCollectionPowers(0, 0);

                        drive.turnSync(Math.toRadians(-70));

                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .back(55)
                                        .build()
                        );
                        hooks.setArmPosition(0);
                        // if moving foundation
                        if (switchStates[2]) {
                            //todo: movements
                            drive.turnSync(Math.toRadians(-100));
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .back(12)
                                            .build()
                            );
                            hooks.clawOpen();
                            sleep(500);
                            hooks.setArmPosition(1);
                            sleep(250);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .forward(10)
                                            .build()
                            );
                            drive.turnSync(Math.toRadians(104));
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .forward(50)
                                            .build()
                            );
                            drive.turnSync(Math.toRadians(47));
                            cal.setCollectionPowers(0.55, 0.55);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .forward(19)
                                            .build()
                            );

                            sleep(250);
                            hooks.clawClose();
                            sleep(750);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .back(19)
                                            .build()
                            );
                            cal.setCollectionPowers(0, 0);
                            drive.turnSync(Math.toRadians(-62));
                            hooks.setArmPosition(0);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .back(30)
                                            .build()
                            );
                            hooks.clawOpen();
                            sleep(250);
                            hooks.setArmPosition(1);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .forward(25)
                                            .build()
                            );
                        }
                        //else just dump the block
                        else {
                            drive.turnSync(Math.toRadians(-100));
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .back(12)
                                            .build()
                            );
                            hooks.clawOpen();
                            sleep(500);
                            hooks.setArmPosition(1);
                            sleep(250);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .forward(10)
                                            .build()
                            );
                            drive.turnSync(Math.toRadians(104));
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .forward(50)
                                            .build()
                            );
                            drive.turnSync(Math.toRadians(47));
                            cal.setCollectionPowers(0.55, 0.55);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .forward(19)
                                            .build()
                            );

                            sleep(250);
                            hooks.clawClose();
                            sleep(750);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .back(19)
                                            .build()
                            );
                            cal.setCollectionPowers(0, 0);
                            drive.turnSync(Math.toRadians(-62));
                            hooks.setArmPosition(0);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .back(30)
                                            .build()
                            );
                            hooks.clawOpen();
                            sleep(250);
                            hooks.setArmPosition(1);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .forward(25)
                                            .build()
                            );
                        }
                        sleep(50000);
                    } else if (valMid == 0) {
                        hooks.setArmPosition(0.94);
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .splineTo(new Pose2d(27, -10, Math.toRadians(-35)))
                                        .build()
                        );
                        cal.setCollectionPowers(0.58, 0.58);
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .forward(12)
                                        .build()
                        );
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .forward(9)
                                        .build()
                        );
                        sleep(500);
                        hooks.setArmPosition(1);
                        hooks.clawClose();
                        sleep(750);
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .back(17)
                                        .build()
                        );
                        cal.setCollectionPowers(0, 0);

                        drive.turnSync(Math.toRadians(-66));

                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .back(85)
                                        .build()
                        );
                        hooks.setArmPosition(0);
                        sleep(1000);
                        //if moving foundation
                        if (switchStates[2]) {
                        }
                        //else drop it and get another
                        else {
                            hooks.clawOpen();
                            sleep(500);
                            hooks.setArmPosition(1);
                            sleep(250);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .strafeRight(3)
                                            .build()
                            );
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .forward(52)
                                            .build()
                            );
                            drive.turnSync(Math.toRadians(50));
                            cal.setCollectionPowers(0.55, 0.55);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .forward(15)
                                            .build()
                            );
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .forward(14)
                                            .build()
                            );

                            sleep(250);
                            hooks.clawClose();
                            sleep(750);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .back(26)
                                            .build()
                            );
                            cal.setCollectionPowers(0, 0);
                            drive.turnSync(Math.toRadians(-55));
                            hooks.setArmPosition(0);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .back(35)
                                            .build()
                            );
                            hooks.clawOpen();
                            sleep(250);
                            hooks.setArmPosition(1);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .forward(25)
                                            .build()
                            );
                        }
                    }
                    //if right block
                    else {
                        hooks.setArmPosition(0.94);
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .splineTo(new Pose2d(29, 5, Math.toRadians(-40)))
                                        .build()
                        );
                        cal.setCollectionPowers(0.55, 0.55);
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .forward(11)
                                        .build()
                        );
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .forward(8)
                                        .build()
                        );
                        sleep(500);
                        hooks.clawClose();
                        sleep(750);
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .back(15)
                                        .build()
                        );
                        cal.setCollectionPowers(0, 0);
                        drive.turnSync(Math.toRadians(-61));
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .back(55)
                                        .build()
                        );
                        hooks.setArmPosition(0);
                        sleep(1000);
                        //if moving foundation
                        if (switchStates[2]) {
                            //todo: movements
                        }
                        //else drop and get another
                        else {
                            hooks.clawOpen();
                            sleep(500);
                            hooks.setArmPosition(1);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .forward(73)
                                            .build()
                            );
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .strafeLeft(20)
                                            .build()
                            );

                            cal.setCollectionPowers(0.55, 0.55);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .forward(6)
                                            .build()
                            );
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .forward(8)
                                            .build()
                            );
                            sleep(500);
                            hooks.clawClose();
                            sleep(750);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .back(12)
                                            .build()
                            );
                            cal.setCollectionPowers(0, 0);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .strafeRight(20)
                                            .build()
                            );
                            hooks.setArmPosition(0);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .back(80)
                                            .build()
                            );
                            hooks.clawOpen();
                            sleep(250);
                            hooks.setArmPosition(1);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .forward(30)
                                            .build()
                            );
                        }
                    }
                }
                //else if red
                else {
                    if (valLeft == 0) {
                        hooks.setArmPosition(0.94);
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .splineTo(new Pose2d(29, -5, Math.toRadians(40)))
                                        .build()
                        );
                        cal.setCollectionPowers(0.55, 0.55);
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .forward(11)
                                        .build()
                        );
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .forward(8)
                                        .build()
                        );
                        sleep(500);
                        hooks.clawClose();
                        sleep(750);
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .back(15)
                                        .build()
                        );
                        cal.setCollectionPowers(0, 0);
                        drive.turnSync(Math.toRadians(61));
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .back(60)
                                        .build()
                        );
                        hooks.setArmPosition(0);
                        sleep(750);
                        //if moving foundation
                        if (switchStates[2]) {
                            //todo: movements
                        }
                        //else drop and get another
                        else {
                            hooks.clawOpen();
                            sleep(500);
                            hooks.setArmPosition(1);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .forward(80)
                                            .build()
                            );
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .strafeRight(25)
                                            .build()
                            );

                            cal.setCollectionPowers(0.55, 0.55);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .forward(6)
                                            .build()
                            );
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .forward(8)
                                            .build()
                            );
                            sleep(500);
                            hooks.clawClose();
                            sleep(750);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .back(12)
                                            .build()
                            );
                            cal.setCollectionPowers(0, 0);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .strafeLeft(25)
                                            .build()
                            );
                            hooks.setArmPosition(0);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .back(80)
                                            .build()
                            );
                            hooks.clawOpen();
                            sleep(250);
                            hooks.setArmPosition(1);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .forward(30)
                                            .build()
                            );
                        }
                    } else if (valMid == 0) {
                        hooks.setArmPosition(0.94);
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .splineTo(new Pose2d(27, 10, Math.toRadians(35)))
                                        .build()
                        );
                        cal.setCollectionPowers(0.58, 0.58);
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .forward(12)
                                        .build()
                        );
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .forward(9)
                                        .build()
                        );
                        sleep(500);
                        hooks.setArmPosition(1);
                        hooks.clawClose();
                        sleep(750);
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .back(17)
                                        .build()
                        );
                        cal.setCollectionPowers(0, 0);

                        drive.turnSync(Math.toRadians(66));

                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .back(85)
                                        .build()
                        );
                        hooks.setArmPosition(0);
                        sleep(1000);
                        //if moving foundation
                        if (switchStates[2]) {
                        }
                        //else drop it and get another
                        else {
                            hooks.clawOpen();
                            sleep(500);
                            hooks.setArmPosition(1);
                            sleep(250);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .strafeRight(3)
                                            .build()
                            );
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .forward(52)
                                            .build()
                            );
                            drive.turnSync(Math.toRadians(-50));
                            cal.setCollectionPowers(0.55, 0.55);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .forward(15)
                                            .build()
                            );
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .forward(14)
                                            .build()
                            );

                            sleep(250);
                            hooks.clawClose();
                            sleep(750);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .back(24)
                                            .build()
                            );
                            cal.setCollectionPowers(0, 0);
                            drive.turnSync(Math.toRadians(55));
                            hooks.setArmPosition(0);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .back(35)
                                            .build()
                            );
                            hooks.clawOpen();
                            sleep(250);
                            hooks.setArmPosition(1);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .forward(25)
                                            .build()
                            );
                        }

                    } else {
                        hooks.setArmPosition(0.94);
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .splineTo(new Pose2d(27, 5, Math.toRadians(30)))
                                        .build()
                        );
                        cal.setCollectionPowers(0.55, 0.55);
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .forward(12)
                                        .build()
                        );
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .forward(9)
                                        .build()
                        );
                        sleep(500);
                        hooks.setArmPosition(1);
                        hooks.clawClose();
                        sleep(750);
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .back(18)
                                        .build()
                        );
                        cal.setCollectionPowers(0, 0);

                        drive.turnSync(Math.toRadians(71));

                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .back(80)
                                        .build()
                        );
                        hooks.setArmPosition(0);
                        // if moving foundation
                        if (switchStates[2]) {

                        }
                        //else just dump the block
                        else {
                            drive.turnSync(Math.toRadians(100));
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .back(13.5)
                                            .build()
                            );
                            hooks.clawOpen();
                            sleep(500);
                            hooks.setArmPosition(1);
                            sleep(250);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .forward(10)
                                            .build()
                            );
                            drive.turnSync(Math.toRadians(-104));
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .forward(48)
                                            .build()
                            );
                            drive.turnSync(Math.toRadians(-48));
                            cal.setCollectionPowers(0.55, 0.55);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .forward(20)
                                            .build()
                            );

                            sleep(250);
                            hooks.clawClose();
                            sleep(750);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .back(19)
                                            .build()
                            );
                            cal.setCollectionPowers(0, 0);
                            drive.turnSync(Math.toRadians(63));
                            hooks.setArmPosition(0);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .back(35)
                                            .build()
                            );
                            hooks.clawOpen();
                            sleep(250);
                            hooks.setArmPosition(1);
                            drive.followTrajectorySync(
                                    drive.trajectoryBuilder()
                                            .forward(30)
                                            .build()
                            );
                        }
                    }
                    sleep(25000);
                }
            }
            //else if building
            else {        
                ExpansionHubServo leftServo, rightServo;

                leftServo = hardwareMap.get(ExpansionHubServo.class, "FoundationServoLeft");
                rightServo = hardwareMap.get(ExpansionHubServo.class, "FoundationServoRight");
                rightServo.setDirection(Servo.Direction.REVERSE);

                if (switchStates[0]) {
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .reverse()
                                    .splineTo(new Pose2d(-32, -20, 0))
                                    .build()
                    );
                }
                else {
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .reverse()
                                    .splineTo(new Pose2d(-32, 20, 0))
                                    .build()
                    );
                }
                leftServo.setPosition(1);
                rightServo.setPosition(1);
                sleep(1000);
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .forward(42)
                                .build()
                );
                leftServo.setPosition(0);
                rightServo.setPosition(0);
                sleep(1000);
                if (switchStates[0]) {
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .strafeLeft(45)
                                    .build()
                    );
                }
                else {
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .strafeRight(45)
                                    .build()
                    );
                }
                if (switchStates[3]){
                    if (switchStates[0])
                    {
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .strafeLeft(35)
                                        .build()
                        );
                    }
                    else {
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .strafeRight(35)
                                        .build()
                        );
                    }
                }
                else {
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .back(28)
                                    .build()
                    );
                    if (switchStates[0]) {
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .strafeLeft(35)
                                        .build()
                        );
                    }
                    else {
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .strafeRight(35)
                                        .build()
                        );
                    }
                }
            }
        }
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

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

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