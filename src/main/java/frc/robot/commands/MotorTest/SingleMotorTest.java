package frc.robot.commands.MotorTest;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.PDHConfig.MotorConfig;
import frc.robot.config.PDHConfig.TestParameters;
import frc.robot.subsystems.PDHManager;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

public class SingleMotorTest extends Command {
    private final MotorConfig motorConfig;
    private final PDHManager pdhManager;
    private final TestParameters params;
    private final Consumer<MotorTestResult> resultCallback;

    private Timer timer;
    private Timer stabilizationTimer;
    private List<Double> currentReadings;
    private Object motor;
    private boolean stabilizationComplete;

    public SingleMotorTest(
        MotorConfig motorConfig,
        PDHManager pdhManager,
        TestParameters params,
        Consumer<MotorTestResult> resultCallback
    ) {
        this.motorConfig = motorConfig;
        this.pdhManager = pdhManager;
        this.params = params;
        this.resultCallback = resultCallback;
        addRequirements(pdhManager);
    }

    @Override
    public void initialize() {
        currentReadings = new ArrayList<>();
        stabilizationComplete = false;

        System.out.println("Testing motor: " + motorConfig.name + " (CAN ID " + motorConfig.canId + ")");

        motor = createMotorController(motorConfig);

        timer = new Timer();
        stabilizationTimer = new Timer();

        startMotor();

        timer.start();
        stabilizationTimer.start();
    }

    @Override
    public void execute() {
        if (!stabilizationComplete && stabilizationTimer.hasElapsed(params.stabilizationTimeMs / 1000.0)) {
            stabilizationComplete = true;
            System.out.println("  Stabilization complete, collecting current readings...");
        }

        if (stabilizationComplete) {
            double current = pdhManager.getCurrent(motorConfig.pdhPort);
            currentReadings.add(current);

            SmartDashboard.putNumber("Test/Live Current/" + motorConfig.name, current);
        }
    }

    @Override
    public void end(boolean interrupted) {
        stopMotor();

        MotorTestResult result;
        if (interrupted) {
            result = new MotorTestResult(
                motorConfig.name,
                motorConfig.canId,
                MotorTestResult.Status.ERROR,
                0.0,
                "Test was interrupted"
            );
        } else {
            result = analyzeResults();
        }

        System.out.println("  " + result);

        resultCallback.accept(result);

        closeMotor();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(params.testDurationMs / 1000.0);
    }

    private Object createMotorController(MotorConfig config) {
        if ("TalonFX".equals(config.motorType)) {
            return new TalonFX(config.canId);
        } else if ("SparkMax".equals(config.motorType)) {
            SparkMax sparkMax = new SparkMax(config.canId, MotorType.kBrushless);
            SparkMaxConfig sparkConfig = new SparkMaxConfig();
            sparkConfig.idleMode(IdleMode.kCoast);
            sparkMax.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
            return sparkMax;
        } else {
            throw new IllegalArgumentException("Unknown motor type: " + config.motorType);
        }
    }

    private void startMotor() {
        if (motor instanceof TalonFX) {
            ((TalonFX) motor).set(params.testVoltage);
        } else if (motor instanceof SparkMax) {
            ((SparkMax) motor).set(params.testVoltage);
        }
    }

    private void stopMotor() {
        if (motor instanceof TalonFX) {
            ((TalonFX) motor).set(0.0);
        } else if (motor instanceof SparkMax) {
            ((SparkMax) motor).set(0.0);
        }
    }

    private void closeMotor() {
        if (motor instanceof TalonFX) {
            ((TalonFX) motor).close();
        } else if (motor instanceof SparkMax) {
            ((SparkMax) motor).close();
        }
    }

    private MotorTestResult analyzeResults() {
        if (currentReadings.isEmpty()) {
            return new MotorTestResult(
                motorConfig.name,
                motorConfig.canId,
                MotorTestResult.Status.ERROR,
                0.0,
                "No current readings collected"
            );
        }

        double avgCurrent = currentReadings.stream()
            .mapToDouble(Double::doubleValue)
            .average()
            .orElse(0.0);

        double stdDev = calculateStdDev(currentReadings, avgCurrent);

        if (avgCurrent < 0.01) {
            return new MotorTestResult(
                motorConfig.name,
                motorConfig.canId,
                MotorTestResult.Status.FAIL,
                avgCurrent,
                "No current draw - check PDH port connection"
            );
        }

        double minThreshold = motorConfig.getMinThreshold(params);
        double maxThreshold = motorConfig.getMaxThreshold(params);

        if (avgCurrent < minThreshold) {
            return new MotorTestResult(
                motorConfig.name,
                motorConfig.canId,
                MotorTestResult.Status.FAIL,
                avgCurrent,
                String.format("Current too low: %.2fA (min: %.2fA)",
                    avgCurrent, minThreshold)
            );
        }

        if (avgCurrent > maxThreshold) {
            return new MotorTestResult(
                motorConfig.name,
                motorConfig.canId,
                MotorTestResult.Status.FAIL,
                avgCurrent,
                String.format("Current too high: %.2fA (max: %.2fA)",
                    avgCurrent, maxThreshold)
            );
        }

        if (stdDev > avgCurrent * 0.2) {
            return new MotorTestResult(
                motorConfig.name,
                motorConfig.canId,
                MotorTestResult.Status.FAIL,
                avgCurrent,
                String.format("Unstable current (stddev: %.2fA)", stdDev)
            );
        }

        return new MotorTestResult(
            motorConfig.name,
            motorConfig.canId,
            MotorTestResult.Status.PASS,
            avgCurrent,
            "All checks passed"
        );
    }

    private double calculateStdDev(List<Double> values, double mean) {
        double variance = values.stream()
            .mapToDouble(val -> Math.pow(val - mean, 2))
            .average()
            .orElse(0.0);
        return Math.sqrt(variance);
    }
}
