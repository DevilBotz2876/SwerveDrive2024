package bhs.devilbotz.utils;

import bhs.devilbotz.BuildConstants;
import edu.wpi.first.wpilibj.RobotBase;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;

/**
 * Determines whether to burn SparkMAX configs to flash.
 *
 * <p>SparkMAX configs should only be burned to flash when the build date changes. This class
 * determines whether the build date has changed since the last time the robot was deployed.
 *
 * @see BuildConstants
 * @see RobotBase
 */
public class SparkMAXBurnManager {
    private static final String buildDateFile = "/home/lvuser/build-date.txt";
    private static boolean shouldBurn = false;

    private SparkMAXBurnManager() {}

    /**
     * @return Whether SparkMAX configs should be burned to flash
     */
    public static boolean shouldBurn() {
        return shouldBurn;
    }

    /**
     * Update whether SparkMAX configs should be burned to flash
     */
    public static void update() {
        if (RobotBase.isSimulation()) {
            System.out.println("[SparkMAXBurnManager] Running in simulation, will not burn SparkMAX flash");
            shouldBurn = false;
            return;
        }

        File file = new File(buildDateFile);
        if (!file.exists()) {

            // No build date file, burn flash
            shouldBurn = true;
        } else {

            // Read previous build date
            String previousBuildDate = "";
            try {
                previousBuildDate =
                        new String(Files.readAllBytes(Paths.get(buildDateFile)),
                                StandardCharsets.UTF_8);
            } catch (IOException e) {
                e.printStackTrace();
            }

            shouldBurn = !previousBuildDate.equals(BuildConstants.BUILD_DATE);
        }

        try {
            FileWriter fileWriter = new FileWriter(buildDateFile);
            fileWriter.write(BuildConstants.BUILD_DATE);
            fileWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

        if (shouldBurn) {
            System.out.println(
                    "[SparkMAXBurnManager] Build date changed, burning SparkMAX flash");
        } else {
            System.out.println(
                    "[SparkMAXBurnManager] Build date unchanged, will not burn SparkMAX flash");
        }
    }
}
