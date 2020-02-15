package auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.util.Angle

object FieldUtil {
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

    fun flip(pose: Pose2d) =
        Pose2d(
            pose.x, -pose.y, Angle.norm(-pose.heading)
        )

    fun getStartingPose(color: Color, side: Side, doesPull: Boolean): Pose2d {
        val pose = Pose2d(
            if (side == FieldUtil.Side.STONE) -33.0 else 33.0, 63.0, if (side == FieldUtil.Side.FOUNDATION && doesPull) Math.toRadians(90.0) else Math.toRadians(270.0)
        )
        return if (color == FieldUtil.Color.RED) flip(pose) else pose
    }
    fun getStonePose(color: Color, index: Int): Pose2d {
        val pose = Pose2d(
            -24.0 - index * 8.0 + (if (index == 5) 8.0 else 0.0),
            26.5 + (if (index == 5) 1.0 else 0.0),
            if (index == 5) Math.toRadians(195.0) else Math.toRadians(230.0)
        )
        return if (color == FieldUtil.Color.RED) flip(pose) else pose
    }
    fun getSkybridgePose(color: Color, side: Side): Pose2d {
        val pose = Pose2d(
            0.0, if (side == FieldUtil.Side.STONE) 37.0 else 60.0, Math.toRadians(180.0)
        )
        return if (color == FieldUtil.Color.RED) flip(pose) else pose
    }
    fun getFoundationPose(color: Color): Pose2d {
        val pose = Pose2d(
            45.0, 29.0, Math.toRadians(97.0)
        )
        return if (color == FieldUtil.Color.RED) flip(pose) else pose
    }
    fun getDroppingPose(color: Color): Pose2d {
        val pose = Pose2d(
            45.0, 40.0, Math.toRadians(180.0)
        )
        return if (color == FieldUtil.Color.RED) flip(pose) else pose
    }
    fun getFoundationReleasePose(color: Color): Pose2d {
        val pose = Pose2d(35.0, 43.0, Math.toRadians(180.0))
        return if (color == FieldUtil.Color.RED) flip(pose) else pose
    }
}