package frc.vision

interface VisionRunner {
    fun periodic()
    val hasTarget: Boolean
    val xOffset: Double
    val yOffset: Double
    val tagArea: Double
    val distance: Double
    fun setLight(mode: Boolean)
}