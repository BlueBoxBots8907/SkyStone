package org.firstinspires.ftc.teamcode.autos.Configuration01;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.hardware.CollectionAndLiftMotors;
import org.firstinspires.ftc.teamcode.hardware.Servos;

@Autonomous(group = "autos")
public class Configuration01Right extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        Servos hooks = new Servos(hardwareMap);
        CollectionAndLiftMotors collectionMotors = new CollectionAndLiftMotors(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;
        //sampling here
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(-23, 27, 0))
                        .build()
        );
        drive.turnSync(Math.toRadians(-140));
        //turn on the collection motors to sample
        collectionMotors.setCollectionPowers(0.6, 0.6);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(29)
                        .build()
        );
        sleep(300);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(22)
                        .build()
        );
        collectionMotors.setCollectionPowers(0, 0);
        drive.turnSync(Math.toRadians(58));
        //drive to the foundation
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(61)
                        .build()
        );

        //go back to get second block (while spitting out the first block)
        collectionMotors.setCollectionPowers(-0.6, -0.6);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(56)
                        .build()
        );
        collectionMotors.setCollectionPowers(0,0);
        drive.turnSync(Math.toRadians(180));
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeLeft(13)
                        .build()
        );
    //get the second block
        collectionMotors.setCollectionPowers(0.6, 0.6);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(12)
                        .build()
        );

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(12)
                        .build()
        );
        collectionMotors.setCollectionPowers(0,0);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(16)
                        .build()
        );
        drive.turnSync(Math.toRadians(175));
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(66)
                        .build()
        );
        //score and park
        collectionMotors.setCollectionPowers(-0.6, -0.6);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(18)
                        .build()
        );
        collectionMotors.setCollectionPowers(0,0);
    }
}