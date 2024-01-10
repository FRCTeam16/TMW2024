package frc.robot.subsystems.util;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;

public class BSPrefs {
    private static final String PREFS_FILE = "/home/lvuser/deploy/bsprefs.csv";
    private static final String OFFSETS_FILE = "/home/lvuser/offsets.csv";
    private static BSPrefs instance;
    private static BSPrefs offsetInstance;
    
    private final String prefsFilename;
    private final Map<String, Double> preferences = new HashMap<String, Double>();

    public static final BSPrefs getInstance() {
        if (instance == null) {
            instance =  new BSPrefs(PREFS_FILE);
        }
        return instance; 
    }

    public static final BSPrefs getOffsetsInstance() {
        if (offsetInstance == null) {
            offsetInstance =  new BSPrefs(OFFSETS_FILE);
        }
        return offsetInstance; 
    } 

    /**
     * Package visible for testing
     * @param filename
     */
    BSPrefs(String filename) {
        this.prefsFilename = filename;
        if (!new File(prefsFilename).exists()) {
            savePreferences();
        }

        try (BufferedReader reader = new BufferedReader(new FileReader(prefsFilename))) {
            String key, value, line = null;
            while ((line = reader.readLine()) != null) {
                String[] parts = line.split(",");
                if (parts.length != 2) {
                    DriverStation.reportWarning("Unexpected preference entry: " + line, false);
                    continue;
                }
                key = parts[0].trim();
                value = parts[1].trim();

                preferences.put(key, Double.parseDouble(value));
            }
        } catch (FileNotFoundException fne) {
            DriverStation.reportError("Unable to find preference file: " + prefsFilename, true);
        } catch (Exception e) {
            DriverStation.reportError("Unexpected error reading preferences: " + e.getMessage(), true);
            throw new RuntimeException("Aborting due to preference file failure", e);
        }
    }

    public double getDouble(String key, double defaultValue) {
        return preferences.getOrDefault(key, defaultValue);
    }

    public void setDouble(String key, double value) {
        preferences.put(key, value);
    }

    public void savePreferences() {
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(prefsFilename))) {
            for (var prefEntry : preferences.entrySet()) {
                writer.write(prefEntry.getKey() + "," + String.valueOf(prefEntry.getValue()) + "\n");
            }
        } catch (Exception e) {
            DriverStation.reportError("Unexpected error writing preferences: " + e.getMessage(), true);
            throw new RuntimeException("Aborting due to preference file write error", e);
        }
    }
}
