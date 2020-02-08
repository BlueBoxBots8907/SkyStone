package org.firstinspires.ftc.teamcode.autos;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.hardware.CollectionAndLiftMotors;
import org.firstinspires.ftc.teamcode.hardware.Servos;

import org.firstinspires.ftc.teamcode.hardware.Switches;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "autos")

@Disabled
public class Autonomous extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = -0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor


    @Override

    public void runOpMode() throws InterruptedException {
        final int rows = 640;
        final int cols = 480;

        OpenCvCamera webcam;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();//open camera
        webcam.setPipeline(new Sampling.StageSwitchingPipeline());//different stages
        webcam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC

        String skystoneLocation = null;

        boolean[] switchStates;

        double splineOffsetX;
        double splineOffsetY;

        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        Servos hooks = new Servos(hardwareMap);
        Sampling detection = new Sampling();
        Switches switches = new Switches(hardwareMap);
        CollectionAndLiftMotors cal = new CollectionAndLiftMotors(hardwareMap);

       switchStates = switches.readSwitches();
       switchStates[2] = false;
       hooks.initPower();
        waitForStart();

        runtime.reset();
        if (isStopRequested()) return;
        hooks.clawOpen();
        hooks.setArmPosition(1);
        //if starting on loading zone
        if(switchStates[1]) {
            //TODO: FIX SAMPLING
            webcam.closeCameraDevice();
            //telemetry.addData("Skystone Location", skystoneLocation);
            telemetry.update();
           skystoneLocation = "right";
            splineOffsetX = -36;
            //if blue
            if(switchStates[0]) {
                splineOffsetY = 61;
                if(skystoneLocation.equalsIgnoreCase("left")){

                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .splineTo(new Pose2d(27, -4, Math.toRadians(-30)))
                                    .build()
                    );
                    cal.setCollectionPowers(0.55, 0.55);
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
                    hooks.clawClose();
                    sleep(750);
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .back(19)
                                    .build()
                    );
                    cal.setCollectionPowers(0,0);

                    drive.turnSync(Math.toRadians(-72));

                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .back(80)
                                    .build()
                    );
                    hooks.setArmPosition(0.5);
                    // if moving foundation
                    if(switchStates[2]){
                        //todo: *good* movements
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
                        cal.setCollectionPowers(0,0);
                        drive.turnSync(Math.toRadians(-62));
                        hooks.setArmPosition(0.5);
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .back(30)
                                        .build()
                        );
                        hooks.clawOpen();
                        sleep(250);
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
                        cal.setCollectionPowers(0,0);
                        drive.turnSync(Math.toRadians(-62));
                        hooks.setArmPosition(0.5);
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .back(30)
                                        .build()
                        );
                        hooks.clawOpen();
                        sleep(250);
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .forward(25)
                                        .build()
                        );
                    }
                }
                else if(skystoneLocation.equalsIgnoreCase("middle")){
                    //todo: put *good* movements here
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .splineTo(new Pose2d(27, -10, Math.toRadians(-35)))
                                    .build()
                    );
                    cal.setCollectionPowers(0.55, 0.55);
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
                    hooks.clawClose();
                    sleep(750);
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .back(15)
                                    .build()
                    );
                    cal.setCollectionPowers(0,0);

                    drive.turnSync(Math.toRadians(-70));

                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .back(85)
                                    .build()
                    );
                    hooks.setArmPosition(0.5);
                    sleep(1000);
                    //if moving foundation
                    if(switchStates[2]){
                        //todo: movements
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
                        drive.turnSync(Math.toRadians(10));
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .forward(48)
                                        .build()
                        );
                        drive.turnSync(Math.toRadians(47));
                        cal.setCollectionPowers(0.55, 0.55);
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .forward(13)
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
                        drive.turnSync(Math.toRadians(-57));
                        hooks.setArmPosition(0.5);
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .back(35)
                                        .build()
                        );
                        hooks.clawOpen();
                        sleep(250);
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .forward(25)
                                        .build()
                        );
                    }
                }
                //if right block
                else {
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
                    cal.setCollectionPowers(0,0);
                    drive.turnSync(Math.toRadians(-55));
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .back(60)
                                    .build()
                    );
                    hooks.setArmPosition(0.5);
                    sleep(750);
                    //if moving foundation
                    if(switchStates[2]){
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
                        cal.setCollectionPowers(0,0);
                        drive.followTrajectorySync(
                                drive.trajectoryBuilder()
                                        .strafeRight(20)
                                        .build()
                        );
                        hooks.setArmPosition(0.5);
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
                                        .forward(25)
                                        .build()
                        );
                    }
                }
            }
            //else red
            else {
                splineOffsetY = -61;
                if(skystoneLocation.equalsIgnoreCase("left")){
                    //todo: put movements here
                    //if moving foundation
                    if(switchStates[2]){
                        //todo: movements
                    }
                    //else nothing happens
                }
                else if(skystoneLocation.equalsIgnoreCase("middle")){
                    //todo: put movements here
                    //if moving foundation
                    if(switchStates[2]){
                        //todo: movements
                    }
                    //else nothing happens
                }
                else {
                    //todo: put movements here
                    //if moving foundation
                    if(switchStates[2]){
                        //todo: movements
                    }
                    //else nothing happens
                }
            }
            sleep(2500);
        }
        //else starting on building zone
        else{
            splineOffsetX = 36;
            //if blue
            if(switchStates[0]){
                splineOffsetY = 61;
                //todo: put movements here
            }
            //else red
            else{
                splineOffsetY = -61;
                //todo: put movements here
            }
        }
        //if park bridge
        if(switchStates[3]) {
         //todo: put movement here
        }
        //else park wall
        else {
         //todo: put movement here
        }
    }
}