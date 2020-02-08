package org.firstinspires.ftc.teamcode.elevator;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Const;
import org.openftc.revextensions2.ExpansionHubMotor;

/*
 * Hardware class for an elevator or linear lift driven by a pulley system.
 */
@Config
public class Elevator {
    private static final MotorConfigurationType MOTOR_CONFIG =
            MotorConfigurationType.getMotorType(GoBILDA5202Series.class);
    private static final double TICKS_PER_REV = MOTOR_CONFIG.getTicksPerRev();

    private static double SPOOL_RADIUS = 18.2/25.4; // in
    private static double GEAR_RATIO = 99.5/3.7; // output (spool) speed / input (motor) speed

    // the operating range of the elevator is restricted to [0, MAX_HEIGHT]
    public static double MAX_HEIGHT = 5.0*180.0/25.4; // in

    private static PIDCoefficients PID = new PIDCoefficients(0, 0, 0);

    public static double MAX_VEL = 25; // in/s
    private static double MAX_ACCEL = 25; // in/s^2
    private static double MAX_JERK = 40; // in/s^3

    private static double kV = 0.022;
    private static double kA = 0;
    private static double kStatic = 0.165;

    private DcMotorEx motorTop, motorBottom;
    private PIDFController controller;
    private MotionProfile profile;
    private NanoClock clock = NanoClock.system();
    private double profileStartTime, desiredHeight = 0;
    private int offset;

    private static double encoderTicksToInches(int ticks) {
        return SPOOL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * SPOOL_RADIUS / 60.0;
    }

    public static double getMaxRpm() {
        return MOTOR_CONFIG.getMaxRPM();
    }

    public Elevator(HardwareMap hardwareMap) {
        motorBottom = hardwareMap.get(DcMotorEx.class, "LiftBottom");
        motorTop = hardwareMap.get(DcMotorEx.class, "LiftTop");

        motorBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // if necessary, reverse the motor so "up" is positive
        motorTop.setDirection(DcMotorSimple.Direction.REVERSE);

        // note: if the elevator is affected by a non-negligible constant force along the direction
        // of motion (e.g., gravity, kinetic friction, or a combination thereof), it may be
        // beneficial to compensate for it with a gravity feedforward
        controller = new PIDFController(PID, kV, kA, kStatic);
        offset = motorTop.getCurrentPosition();
    }

    public boolean isBusy() {
        return profile != null && (clock.seconds() - profileStartTime) <= profile.duration();
    }

    public void setHeight(double height) {
        height = Math.min(Math.max(0, height), MAX_HEIGHT);

        double time = clock.seconds() - profileStartTime;
        MotionState start = isBusy() ? profile.get(time) : new MotionState(desiredHeight, 0, 0, 0);
        MotionState goal = new MotionState(height, 0, 0, 0);
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                start, goal, MAX_VEL, MAX_ACCEL, MAX_JERK
        );
        profileStartTime = clock.seconds();

        desiredHeight = height;
    }

    public double getCurrentHeight() {
        return encoderTicksToInches(motorTop.getCurrentPosition() - offset);
    }

    public void update() {
        double power;
        double currentHeight = getCurrentHeight();
        if (isBusy()) {
            // following a profile
            double time = clock.seconds() - profileStartTime;
            MotionState state = profile.get(time);
            controller.setTargetPosition(state.getX());
            power = controller.update(currentHeight, state.getV(), state.getA());
        } else {
            // just hold the position
            controller.setTargetPosition(desiredHeight);
            power = controller.update(currentHeight);
        }
        setPower(power);
    }

    public void setPower(double power) {
        motorTop.setPower(power);
        motorBottom.setPower(power);
    }
}