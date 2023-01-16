import edu.wpi.first.deployutils.deploy.artifact.FileTreeArtifact
import edu.wpi.first.gradlerio.deploy.roborio.FRCJavaArtifact
import edu.wpi.first.gradlerio.deploy.roborio.RoboRIO
import edu.wpi.first.gradlerio.wpi.dependencies.tools.ToolInstallTask
import edu.wpi.first.toolchain.NativePlatforms
import org.jetbrains.kotlin.gradle.tasks.KotlinCompile

plugins {
    id("com.github.johnrengelman.shadow")
    kotlin("jvm")
    id("edu.wpi.first.GradleRIO")
}

group = "org.aztechs"
version = "2023"


val fatJar by tasks.register<Jar>("fatJar") {
    dependsOn(tasks.named("shadowJar"))

    group = "build"
    description = """
        Setting up my Jar File. In this case, adding all libraries into the main jar ('fat jar')
        in order to make them all available at runtime. Also adding the manifest so WPILib
        knows where to look for our Robot Class.
    """.trimIndent()


    manifest {
        attributes["Main-Class"] = "frc.robot.MainKt"
    }

    from(
        configurations
            .runtimeClasspath
            .get()
    )
    duplicatesStrategy = DuplicatesStrategy.INCLUDE
    with(tasks.jar.get() as CopySpec)
}

tasks {
    jar {
        manifest {
            attributes["Main-Class"] = "frc.robot.MainKt"
        }
    }

    test {
        useJUnitPlatform()
    }

    withType<KotlinCompile> {
        kotlinOptions {

        }
    }

    ToolInstallTask.setToolsFolder(project.rootProject.file(".wpilib/tools/"))
    create<JavaExec>("launchShuffleboard") {
        group = "tools"
        classpath = files(project.rootProject.file(".wpilib/tools/ShuffleBoard.jar"))
    }

    create<JavaExec>("launchSmartdash") {
        group = "tools"
        classpath = files(project.rootProject.file(".wpilib/tools/ShuffleBoard.jar"))
    }
}


wpi {
    // Simulation configuration (e.g. environment variables).
    with(sim) {
        addGui().defaultEnabled.set(true)
        addDriverstation()
    }

    with(java) {
        // Configure jar and deploy tasks
        configureExecutableTasks(fatJar)
        configureTestTasks(tasks.test.get())
        // Set to true to use debug for JNI.
        debugJni.set(false)
    }
}

// Define my targets (RoboRIO) and artifacts (deployable files)
// This is added by GradleRIO's backing project DeployUtils.
deploy {
    targets {
        create<RoboRIO>(name = "roborio") {
            // Team number is loaded either from the .wpilib/wpilib_preferences.json
            // or from command line. If not found an exception will be thrown.
            // You can use getTeamOrDefault(team) instead of getTeamNumber if you
            // want to store a team number in this file.
            team = project.frc.teamNumber
//            debug = project.frc.getDebugOrDefault(false)
            directory = "/home/lvuser/deploy"
            this.artifacts {
                create<FRCJavaArtifact>("frcJava") {
                    dependsOn(fatJar)
                    setJarTask(fatJar)
                }

                create<FileTreeArtifact>("frcStaticFileDeploy") {
                    files(project.fileTree("src/main/deploy"))
                }
            }
        }
    }
}

dependencies {
    with(wpi.java) {
        deps.wpilib().forEach { implementation(it.get()) }
        vendor.java().forEach { implementation(it.get()) }

        deps.wpilibJniDebug(NativePlatforms.roborio).forEach { "roborioDebug"(it.get()) }
        vendor.jniDebug(NativePlatforms.roborio).forEach { "roborioDebug"(it.get()) }

        deps.wpilibJniRelease(NativePlatforms.roborio).forEach { "roborioRelease"(it.get()) }
        vendor.jniRelease(NativePlatforms.roborio).forEach { "roborioRelease"(it.get()) }

        deps.wpilibJniDebug(NativePlatforms.roborio).forEach { nativeDebug(it) }
        vendor.jniDebug(NativePlatforms.roborio).forEach { nativeDebug(it) }

        deps.wpilibJniRelease(NativePlatforms.roborio).forEach { nativeRelease(it) }
        vendor.jniRelease(NativePlatforms.roborio).forEach { nativeRelease(it) }
    }

    wpi.sim.enableRelease().forEach { simulationRelease(it) }

    implementation(platform("org.junit:junit-bom:5.8.2"))
    testImplementation("org.junit.jupiter:junit-jupiter-api")
    testImplementation("org.junit.jupiter:junit-jupiter-params")
    testRuntimeOnly("org.junit.jupiter:junit-jupiter-engine")

    testImplementation("io.kotest:kotest-runner-junit5:5.5.4")
    testImplementation("io.kotest:kotest-assertions-core:5.5.4")
    testImplementation("io.mockk:mockk:1.13.2")
}
