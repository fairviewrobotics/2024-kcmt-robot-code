package frc.robot.utils;

import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;

public class ConfigUtils {
    private static ConfigUtils INSTANCE;

    private File configPath = Path.of(Filesystem.getDeployDirectory().toPath().toString(), "config.json").toFile();

    public ConfigUtils getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ConfigUtils();
        }

        return INSTANCE;
    }

    public ConfigUtils() {
        try {
            if (configPath.createNewFile()) {
                
            }
        } catch (IOException e) {
            System.out.println("[WARN] Failed to create config file: " + e);
        }
     }
}
