package org.firstinspires.ftc.teamcode.autos.Configuration02;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.hardware.Servos;
import org.firstinspires.ftc.teamcode.hardware.Switches;
import org.openftc.revextensions2.ExpansionHubServo;


@Autonomous(name= "Autonomous(Foundation Side)", group="Sky autonomous")
public class Configuration02 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        Servos hooks = new Servos(hardwareMap);
        Switches switches = new Switches(hardwareMap);
        boolean[] switchStates = switches.readSwitches();
        ExpansionHubServo leftServo, rightServo;

        leftServo = hardwareMap.get(ExpansionHubServo.class, "FoundationServoLeft");
        rightServo = hardwareMap.get(ExpansionHubServo.class, "FoundationServoRight");
        rightServo.setDirection(Servo.Direction.REVERSE);
        waitForStart();

        if (isStopRequested()) return;
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
