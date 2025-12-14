package frc.robot.config;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.Map;

public class PDHConfig {
    public PDH pdh;
    public Map<String, MotorConfig> motorPorts;
    public TestParameters testParameters;

    private static PDHConfig instance = null;

    public static class PDH {
        public int moduleId;
    }

    public static class MotorConfig {
        public int canId;
        public int pdhPort;
        public String motorType;
        public String subsystem;
        public String name;
        public Double currentThresholdMin;
        public Double currentThresholdMax;

        public double getMinThreshold(TestParameters defaults) {
            return currentThresholdMin != null ? currentThresholdMin : defaults.currentThresholdMin;
        }

        public double getMaxThreshold(TestParameters defaults) {
            return currentThresholdMax != null ? currentThresholdMax : defaults.currentThresholdMax;
        }
    }

    public static class TestParameters {
        public double testVoltage;
        public int testDurationMs;
        public double currentThresholdMin;
        public double currentThresholdMax;
        public int stabilizationTimeMs;
    }

    public static PDHConfig load() {
        if (instance != null) {
            return instance;
        }

        try {
            Path configPath = Filesystem.getDeployDirectory().toPath().resolve("pdh-config.json");
            File configFile = configPath.toFile();

            ObjectMapper mapper = new ObjectMapper();
            instance = mapper.readValue(configFile, PDHConfig.class);

            if (instance == null) {
                throw new RuntimeException("Failed to parse pdh-config.json");
            }

            System.out.println("PDH Config loaded successfully:");
            System.out.println("  PDH Module ID: " + instance.pdh.moduleId);
            System.out.println("  Motors configured: " + instance.motorPorts.size());

            return instance;
        } catch (IOException e) {
            System.err.println("ERROR: Failed to load pdh-config.json");
            e.printStackTrace();
            throw new RuntimeException("Could not load PDH configuration", e);
        }
    }

    public int getPdhModuleId() {
        return pdh.moduleId;
    }

    public Map<String, MotorConfig> getMotorPorts() {
        return motorPorts;
    }

    public TestParameters getTestParameters() {
        return testParameters;
    }

    public MotorConfig getMotorByCanId(int canId) {
        for (MotorConfig motor : motorPorts.values()) {
            if (motor.canId == canId) {
                return motor;
            }
        }
        return null;
    }
}