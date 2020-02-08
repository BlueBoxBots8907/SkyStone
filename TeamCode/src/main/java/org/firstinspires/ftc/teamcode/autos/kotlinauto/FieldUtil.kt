package org.firstinspires.ftc.teamcode.autos.kotlinauto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.util.Angle

fun Double.toRadians(): Double = Math.toRadians(this)

public object FieldUtil {
    enum class Color {
        RED {
            override operator fun not() = BLUE
        },
        BLUE {
            override fun not() = RED
        };
        abstract operator fun not(): Color
    }
    enum class Side {
        STONE {
            override operator fun not() = FOUNDATION
        },
        FOUNDATION {
            override fun not() = STONE
        };
        abstract operator fun not(): Side
    }

    @JvmStatic
    fun flip(pose: Pose2d) =
            Pose2d(
                    pose.x, -pose.y, Angle.norm(-pose.heading)
            )

    @JvmStatic
    fun getStartingPose(color: Color, side: Side, doesPull: Boolean): Pose2d {
        val pose = Pose2d(
                if (side == FieldUtil.Side.STONE) -33.0 else 33.0, 63.0, if (side == FieldUtil.Side.FOUNDATION && doesPull) Math.toRadians(90.0) else Math.toRadians(270.0)
        )
        return if (color == FieldUtil.Color.RED) flip(pose) else pose
    }

    @JvmStatic
    fun getStonePose(color: Color, index: Int): Pose2d {
        val pose = Pose2d(
                -26.0 - index * 8.0 - 1.0,
                36.0,
                Math.toRadians(270.0)
        )
        return if (color == FieldUtil.Color.RED) flip(pose) else pose
    }

    @JvmStatic
    fun getStonePoseFive(color: Color): Pose2d {
        val pose = Pose2d(-60.0, 27.0, Math.toRadians(225.0))
        return if (color == FieldUtil.Color.RED) flip(pose) else pose
    }

    @JvmStatic @JvmOverloads
    fun getSkybridgePose(color: Color, side: Side, avoidBridge: Boolean = false): Pose2d {
        val pose = Pose2d(
                0.0, if (side == FieldUtil.Side.STONE) 40.0 + (if (avoidBridge) 5.0 else 0.0) else 60.0, Math.toRadians(180.0)
        )
        return if (color == FieldUtil.Color.RED) flip(pose) else pose
    }

    @JvmStatic
    fun getFoundationPose(color: Color): Pose2d {
        val pose = Pose2d(
                45.0, 30.0, Math.toRadians(97.0)
        )
        return if (color == FieldUtil.Color.RED) flip(pose) else pose
    }

    @JvmStatic
    fun getDroppingPose(color: Color): Pose2d {
        val pose = Pose2d(
                45.0, 42.0, Math.toRadians(180.0)
        )
        return if (color == FieldUtil.Color.RED) flip(pose) else pose
    }

    @JvmStatic
    fun getFoundationReleasePose(color: Color): Pose2d {
        val pose = Pose2d(33.0, 46.0, Math.toRadians(180.0))
        return if (color == FieldUtil.Color.RED) flip(pose) else pose
    }
}