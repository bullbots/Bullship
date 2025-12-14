package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.PDHConfig;

public class PDHManager extends SubsystemBase {
    private final PowerDistribution pdh;
    private final PDHConfig config;

    public PDHManager() {
        config = PDHConfig.load();
        pdh = new PowerDistribution(config.getPdhModuleId(), ModuleType.kRev);

        System.out.println("PDHManager initialized with PDH module ID: " + config.getPdhModuleId());
    }

    /**
     * Get the current draw from a specific PDH port
     * @param port The PDH port number (0-23)
     * @return Current in amps
     */
    public double getCurrent(int port) {
        return pdh.getCurrent(port);
    }

    /**
     * Get the battery voltage from the PDH
     * @return Voltage in volts
     */
    public double getVoltage() {
        return pdh.getVoltage();
    }

    /**
     * Get the total current draw across all PDH channels
     * @return Total current in amps
     */
    public double getTotalCurrent() {
        return pdh.getTotalCurrent();
    }

    /**
     * Get the PDH temperature
     * @return Temperature in Celsius
     */
    public double getTemperature() {
        return pdh.getTemperature();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("PDH Voltage", getVoltage());
        SmartDashboard.putNumber("PDH Total Current", getTotalCurrent());
        SmartDashboard.putNumber("PDH Temperature", getTemperature());
    }
}