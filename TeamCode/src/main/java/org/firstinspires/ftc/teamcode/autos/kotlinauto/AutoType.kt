package org.firstinspires.ftc.teamcode.autos.kotlinauto

public data class AutoType(@JvmField var color: FieldUtil.Color,
                           @JvmField var side: FieldUtil.Side,
                           @JvmField var parkingSide: FieldUtil.Side,
                           @JvmField var skystoneIdx: Int,
                           @JvmField var doesPull: Boolean
)