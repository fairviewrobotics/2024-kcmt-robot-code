package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTableListenerPoller;
import edu.wpi.first.wpilibj.Filesystem;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import java.io.*;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Scanner;

public class ConfigUtils {
    private static ConfigUtils INSTANCE;

    private final File configFile = Path.of(Filesystem.getDeployDirectory().toPath().toString(), "tuning.json").toFile();

    private final HashMap<String, Double> data = new HashMap<>();

    private JSONObject json;

    /**
     * Get the instance of the config utils
     * @return Instance of config utils
     */
    public static ConfigUtils getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ConfigUtils();
        }

        return INSTANCE;
    }

    /**
     * Util class to allow for good network table tuning
     * TODO: Integrate better with network tables with (Maybe with {@link edu.wpi.first.networktables.NetworkTableListener})
     */
    public ConfigUtils() {
        try {
            if (configFile.createNewFile()) {
                System.out.println("[INFO] Created tuning file");
                this.json = this.getDefault();
                this.saveConfig();
            }
        } catch (IOException e) {
            System.out.println("[WARN] Failed to create config file: " + e);
        }

        this.json = parseConfig();
    }

    /**
     * Get the default settings (used to create the json file if it does not exist)
     * @return A default json object
     */
    @SuppressWarnings("unchecked")
    private JSONObject getDefault() {
        JSONObject defaultSettings = new JSONObject();

        // INTAKE
        defaultSettings.put("intake_notein_speed", 0.0);
        defaultSettings.put("intake_shoot_speed", 0.0);

        // SHOOTER
        defaultSettings.put("shooter_speaker", 0.0);
        defaultSettings.put("shooter_high_pass", 0.0);
        defaultSettings.put("shooter_low_pass", 0.0);
        defaultSettings.put("shooter_amp", 0.0);

        // ARM
        defaultSettings.put("placeholder", 0.0);

        return defaultSettings;
    }

    /**
     * Get a double from the config
     * @param key The key in the json
     * @param defaultValue A default value in case we fail to get the key
     * @return The value from the key
     */
    public double getDouble(String key, double defaultValue) {
        double res = defaultValue;
        try {
            res = (double) this.json.get(key);
        } catch (Exception e) {
            System.out.println("[WARN] Failed to get " + key + ": " + e);
        }

        return res;
     }

    /**
     * Set a double value
     * @param key The key for the json file
     * @param value The value to set
     */
     @SuppressWarnings("unchecked")
     public void setDouble(String key, double value) {
        this.json.put(key, value);
     }


    /**
     * Save the config to the config file location
     */
    public void saveConfig() {
         try {
             FileWriter fileWrite = new FileWriter(this.configFile);
             this.json.writeJSONString(fileWrite);
         } catch (IOException e) {
             System.out.println("[WARN] Failed to save file: " + configFile);
         }
    }


    /**
     * Parse the config file
     * @return The parsed config as a {@link JSONObject}
     */
    private JSONObject parseConfig() {
        StringBuilder data = new StringBuilder();
        Object parsedObject = new Object();

        try {
            Scanner scanner = new Scanner(this.configFile);
            while (scanner.hasNextLine()) {
                data.append(scanner.nextLine());
            }
            scanner.close();

            JSONParser parser = new JSONParser();
            parsedObject = parser.parse(data.toString());
        } catch (FileNotFoundException | ParseException e) {
            System.out.println("[ERROR] An error occurred: " + e);
        }
    return (JSONObject) parsedObject;
    }
}
