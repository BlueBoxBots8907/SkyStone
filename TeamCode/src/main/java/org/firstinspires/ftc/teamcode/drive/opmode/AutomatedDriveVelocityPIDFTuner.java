package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.BASE_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.getRawMaxRpm;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.rpmToVelocity;


/*
 * Op mode for computing kP, kD, kI, and kF for velocity PIDF. The process is as follows:
 *   Perform a simple straight movement and record the maximum velocity.
 *   kF = 32767.0 (ticks) / maxV
 *   kP = 0.1 * kF
 *   kI = 0.1 * kP
 *   kD = 0
 * These procedures are taken from the [FIRST Global velocity PIDF tuning guide](https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit?usp=sharing)
 * The P, I, D terms are far too conservative for most drivebases and should be tweaked later.
 */
@Config
@Autonomous(group = "drive")
public class AutomatedDriveVelocityPIDFTuner extends LinearOpMode {
    private static double DISTANCE = 100;

    private double getMaxVelocity() {
        return rpmToVelocity(getRawMaxRpm());
    }

    private MotionProfile generateProfile() {
        return MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0, 0, 0, 0),
                new MotionState(DISTANCE, 0, 0, 0),
                getMaxVelocity(),
                BASE_CONSTRAINTS.maxAccel,
                0 // makes computing if it can reach the target much easier.
        );
    }

    private boolean canReachPeakVelocity() {
        return DISTANCE >= getMaxVelocity() * getMaxVelocity() / BASE_CONSTRAINTS.maxAccel;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        if (!RUN_USING_ENCODER) {
            RobotLog.setGlobalErrorMsg("PIDF constants don't need to be tuned " +
                    "when not using the built-in drive motor velocity PID.");
            while (!isStopRequested()) {
                idle();
            }
            return;
        }

        if (!canReachPeakVelocity()) {
            RobotLog.setGlobalWarningMessage("The test could not reach the peak velocity needed for tuning." +
                    "Set a higher acceleration constraint and/or increase distance");
            while (!isStopRequested()) {
                idle();
            }
            return;
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        NanoClock clock = NanoClock.system();

        telemetry.addLine(Misc.formatInvariant(
                "Place your robot on the field with at least %.2f in of room in front", DISTANCE));
        telemetry.addLine("Press play to begin the automated PIDF tuning routine");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.clearAll();
        telemetry.addLine("Running...");
        telemetry.update();

        MotionProfile moveProfile = generateProfile();

        double profileStart = clock.seconds();

        double maxVelocity = 0;

        while (!isStopRequested()) {
            // calculate and set the motor power
            double profileTime = clock.seconds() - profileStart;

            if (profileTime > moveProfile.duration()) {
                telemetry.addLine("Testing completed!");
                telemetry.update();
                break;
            }

            MotionState motionState = moveProfile.get(profileTime);
            double targetPower = motionState.getV() / getMaxVelocity();
            drive.setDrivePower(new Pose2d(targetPower, 0, 0));

            List<Double> velocities = drive.getWheelVelocities();

            for (double velocity : velocities) {
                if (velocity > maxVelocity) {
                    maxVelocity = velocity;
                }
            }

            // update telemetry
            telemetry.addData("targetPower", targetPower);
            telemetry.addData("targetVelocity", motionState.getV());
            telemetry.addData("maximumRecordedVelocity", maxVelocity);
            for (int i = 0; i < velocities.size(); i++) {
                telemetry.addData("velocity" + i, velocities.get(i));
                telemetry.addData("error" + i, motionState.getV() - velocities.get(i));
            }
            telemetry.update();
        }

        drive.setDrivePower(new Pose2d());

        double kF = encoderTicksToInches(32767.0) / maxVelocity;
        double kP = 0.1 * kF;
        double kI = 0.1 * kP;
        double kD = 0;

        while (!isStopRequested()) {
            telemetry.clearAll();
            telemetry.addData("maxVelocity", maxVelocity);
            telemetry.addData("kP", kP);
            telemetry.addData("kI", kI);
            telemetry.addData("kD", kD);
            telemetry.addData("kF", kF);
            telemetry.update();
            idle();
        }
    }
}