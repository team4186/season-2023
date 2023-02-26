package frc.vision

interface VisionRunner {
    fun periodic()
    val hasTarget: Boolean
    val xOffset: Double
    val yOffset: Double
    val targetArea: Double
    val distance: Double
    fun setLight(mode: Boolean)
}