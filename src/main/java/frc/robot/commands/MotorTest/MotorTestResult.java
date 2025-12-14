package frc.robot.commands.MotorTest;

public class MotorTestResult {
    public enum Status {
        PASS,
        FAIL,
        ERROR
    }

    private final String motorName;
    private final int canId;
    private final Status status;
    private final double measuredCurrent;
    private final String failureReason;
    private final long timestamp;

    public MotorTestResult(String motorName, int canId, Status status, double measuredCurrent, String failureReason) {
        this.motorName = motorName;
        this.canId = canId;
        this.status = status;
        this.measuredCurrent = measuredCurrent;
        this.failureReason = failureReason;
        this.timestamp = System.currentTimeMillis();
    }

    public String getMotorName() {
        return motorName;
    }

    @SuppressWarnings("unused")
    public int getCanId() {
        return canId;
    }

    public Status getStatus() {
        return status;
    }

    public double getMeasuredCurrent() {
        return measuredCurrent;
    }

    public String getFailureReason() {
        return failureReason;
    }

    @SuppressWarnings("unused")
    public long getTimestamp() {
        return timestamp;
    }

    @Override
    public String toString() {
        if (status == Status.PASS) {
            return String.format("%s (ID %d): PASS (%.2fA)", motorName, canId, measuredCurrent);
        } else {
            return String.format("%s (ID %d): %s (%.2fA) - %s",
                motorName, canId, status, measuredCurrent, failureReason);
        }
    }
}