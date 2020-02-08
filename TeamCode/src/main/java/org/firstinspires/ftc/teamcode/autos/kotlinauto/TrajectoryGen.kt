//import org.firstinspires.ftc.teamcode.autos.auto.AutoType
//import org.firstinspires.ftc.teamcode.autos.auto.FieldUtil
//import com.acmerobotics.roadrunner.geometry.Pose2d
//import com.acmerobotics.roadrunner.geometry.Vector2d
//import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator
//import com.acmerobotics.roadrunner.path.heading.WiggleInterpolator
//import com.acmerobotics.roadrunner.trajectory.Trajectory
//import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
//import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
//import java.lang.reflect.Field
//
//object TrajectoryGen {
//    private val constraints = DriveConstraints(30.0, 30.0, 0.0, 180.0.toRadians, 180.0.toRadians, 0.0.toRadians)
//    // change this to change the mode
//    private val autoType = AutoType(FieldUtil.Color.RED, FieldUtil.Side.STONE, 1)
//    private var startPose = FieldUtil.getStartingPose(autoType.color, autoType.side)
//    private var trajectories = mutableListOf<TrajectoryBuilder>()
//
//    var isDown = false
//
//    private fun makeTrajectory(autoType: AutoType) {
//        if (autoType.side == FieldUtil.Side.STONE) {
//            builder() // skystone 1
//                .splineTo(FieldUtil.getStonePose(autoType.color, autoType.skystoneIdx))
//            builder() // foundation pull + skystone 1 deposit
//                .reverse()
//                .splineTo(FieldUtil.getSkybridgePose(autoType.color, autoType.side))
//                .splineTo(FieldUtil.getFoundationPose(autoType.color))
//            builder()
//                .splineTo(Pose2d(30.0, if (autoType.color == FieldUtil.Color.RED) -45.0 else 45.0, 180.0.toRadians))
//            builder() // foundation release
//                .back(15.0)
//            builder() // skystone 2
//                .splineTo(FieldUtil.getSkybridgePose(autoType.color, autoType.side))
//                // .splineTo(Pose2d(-40.0, if (autoType.color == FieldUtil.Color.RED) -35.0 else 35.0, 180.0.toRadians))
//                .splineTo(FieldUtil.getStonePose(autoType.color,autoType.skystoneIdx + 3))
//            builder() // skystone 2 deposit
//                .reverse()
//                .splineTo(FieldUtil.getSkybridgePose(autoType.color, autoType.side))
//                .splineTo(FieldUtil.getDroppingPose(autoType.color))
//            builder() // navigating/parking
//                .splineTo(FieldUtil.getSkybridgePose(autoType.color, autoType.side))
//        } else {
//            builder() // foundation pull + skystone 1 deposit
//                .reverse()
//                .splineTo(FieldUtil.getFoundationPose(autoType.color))
//            builder()
//                .splineTo(Pose2d(30.0, if (autoType.color == FieldUtil.Color.RED) -45.0 else 45.0, 180.0.toRadians))
//            builder() // foundation release
//                .back(15.0)
//            builder() // navigating/parking
//                .splineTo(FieldUtil.getSkybridgePose(autoType.color, autoType.side))
//        }
//    }
//
//    fun createTrajectories(): List<Trajectory> {
//        val autos = mutableListOf<MutableList<TrajectoryBuilder>>();
//
////        FieldUtil.Color.values().forEach { color ->
////            FieldUtil.Side.values().forEach { side ->
////                for (i in 0..2) {
////                    val autoType = AutoType(color, side, i, true)
////                    startPose = FieldUtil.getStartingPose(autoType.color, autoType.side)
////                    makeTrajectory(autoType)
////                    autos.add(trajectories)
////                    trajectories = mutableListOf()
////                }
////            }
////        }
//
//        makeTrajectory(autoType)
//        autos.add(trajectories)
//        return autos.flatten().map { it.build() }
//    }
//
//    fun builder(): TrajectoryBuilder {
//        val lastPose = if (trajectories.size > 0) trajectories.last().build()[trajectories.last().build().duration()] else startPose
//        val trajectory = TrajectoryBuilder(lastPose, constraints)
//        trajectories.add(trajectory)
//        return trajectory
//    }
//
//    fun drawOffbounds() {
//        //GraphicsUtil.fillRect(FieldUtil.getSkybridgePose(autoType.color, !autoType.side).vec(), 18.0, 18.0) // robot against the wall
//    }
//}
//
//val Double.toRadians get() = (Math.toRadians(this))