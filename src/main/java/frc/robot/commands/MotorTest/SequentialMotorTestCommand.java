package frc.robot.commands.MotorTest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.config.PDHConfig;
import frc.robot.config.PDHConfig.MotorConfig;
import frc.robot.subsystems.PDHManager;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class SequentialMotorTestCommand extends SequentialCommandGroup {
    private final List<MotorTestResult> results;
    private final int totalMotors;

    public SequentialMotorTestCommand(PDHManager pdhManager) {
        results = new ArrayList<>();

        PDHConfig config = PDHConfig.load();
        Map<String, MotorConfig> motorPorts = config.getMotorPorts();
        totalMotors = motorPorts.size();
        int currentMotorIndex = 0;

        System.out.println("===== Starting Motor Test Sequence =====");
        System.out.println("Total motors to test: " + totalMotors);

        SmartDashboard.putString("Test/Status", "RUNNING");
        SmartDashboard.putNumber("Test/Total Motors", totalMotors);
        SmartDashboard.putNumber("Test/Motors Tested", 0);
        SmartDashboard.putNumber("Test/Motors Passed", 0);
        SmartDashboard.putNumber("Test/Motors Failed", 0);

        for (Map.Entry<String, MotorConfig> entry : motorPorts.entrySet()) {
            MotorConfig motorConfig = entry.getValue();
            final int motorIndex = currentMotorIndex;

            addCommands(
                new InstantCommand(() -> updateDashboardStart(motorConfig, motorIndex)),

                new WaitCommand(0.5),

                new SingleMotorTest(
                    motorConfig,
                    pdhManager,
                    config.getTestParameters(),
                    result -> {
                        results.add(result);
                        postResultToDashboard(result);
                    }
                ),

                new WaitCommand(0.3)
            );

            currentMotorIndex++;
        }

        addCommands(
            new InstantCommand(this::displayTestSummary)
        );
    }

    private void updateDashboardStart(MotorConfig motorConfig, int index) {
        System.out.println("\n[" + (index + 1) + "/" + totalMotors + "] Testing: " + motorConfig.name);

        SmartDashboard.putString("Test/Current Motor", motorConfig.name);
        SmartDashboard.putNumber("Test/Current Motor CAN ID", motorConfig.canId);
        SmartDashboard.putNumber("Test/Current Motor PDH Port", motorConfig.pdhPort);
        SmartDashboard.putString("Test/Current Motor Type", motorConfig.motorType);
        SmartDashboard.putString("Test/Progress", (index + 1) + " / " + totalMotors);
    }

    private void postResultToDashboard(MotorTestResult result) {
        SmartDashboard.putString("Test/Result/" + result.getMotorName(), result.getStatus().toString());
        SmartDashboard.putNumber("Test/Current/" + result.getMotorName(), result.getMeasuredCurrent());

        if (result.getStatus() != MotorTestResult.Status.PASS) {
            SmartDashboard.putString("Test/Failure/" + result.getMotorName(), result.getFailureReason());
        }

        long passed = results.stream()
            .filter(r -> r.getStatus() == MotorTestResult.Status.PASS)
            .count();
        long failed = results.stream()
            .filter(r -> r.getStatus() == MotorTestResult.Status.FAIL)
            .count();

        SmartDashboard.putNumber("Test/Motors Tested", results.size());
        SmartDashboard.putNumber("Test/Motors Passed", passed);
        SmartDashboard.putNumber("Test/Motors Failed", failed);
    }

    private void displayTestSummary() {
        long passed = results.stream()
            .filter(r -> r.getStatus() == MotorTestResult.Status.PASS)
            .count();
        long failed = results.stream()
            .filter(r -> r.getStatus() == MotorTestResult.Status.FAIL)
            .count();
        long errors = results.stream()
            .filter(r -> r.getStatus() == MotorTestResult.Status.ERROR)
            .count();

        String summary = passed + " / " + results.size() + " motors passed";
        SmartDashboard.putString("Test/Summary", summary);
        SmartDashboard.putString("Test/Status", "COMPLETE");

        System.out.println("\n===== Motor Test Complete =====");
        System.out.println("Total: " + results.size());
        System.out.println("Passed: " + passed);
        System.out.println("Failed: " + failed);
        System.out.println("Errors: " + errors);

        if (failed > 0 || errors > 0) {
            System.out.println("\nFailed/Error Motors:");
            results.stream()
                .filter(r -> r.getStatus() != MotorTestResult.Status.PASS)
                .forEach(r -> System.out.println("  " + r));
        }

        System.out.println("===================================\n");
    }
}
