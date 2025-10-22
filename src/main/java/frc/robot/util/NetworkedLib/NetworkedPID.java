package frc.robot.util.NetworkedLib;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Exactly like the original PIDController
 * Except it allows for quick tuning via network modified values
 */
public class NetworkedPID extends PIDController {
    private static int instanceCount;

    private String name;

    private DoubleTopic kpTopic;
    private DoubleTopic kiTopic;
    private DoubleTopic kdTopic;

    private DoublePublisher kpPub;
    private DoublePublisher kiPub;
    private DoublePublisher kdPub;

    private DoubleSubscriber kpSub;
    private DoubleSubscriber kiSub;
    private DoubleSubscriber kdSub;

    /**
     * Allocates a PIDController with the given constants for kp, ki, and kd and a
     * default period of 0.02 seconds.
     *
     * @param kp The proportional coefficient.
     * @param ki The integral coefficient.
     * @param kd The derivative coefficient.
     * @throws IllegalArgumentException if kp &lt; 0
     * @throws IllegalArgumentException if ki &lt; 0
     * @throws IllegalArgumentException if kd &lt; 0
     */
    public NetworkedPID(double kp, double ki, double kd) {
        super(kp, ki, kd);
        instanceCount++;

        this.name = "NetworkedPIDController" + instanceCount;

        setupNT(kp, ki, kd);
    }

    /**
     * Allocates a PIDController with the given constants for kp, ki, and kd and a
     * default period of 0.02 seconds. Takes in a name for the NetworkTable
     *
     * @param kp   The proportional coefficient.
     * @param ki   The integral coefficient.
     * @param kd   The derivative coefficient.
     * @param name The name to use in the NetworkTable
     * @throws IllegalArgumentException if kp &lt; 0
     * @throws IllegalArgumentException if ki &lt; 0
     * @throws IllegalArgumentException if kd &lt; 0
     */
    public NetworkedPID(double kp, double ki, double kd, String name) {
        super(kp, ki, kd);
        instanceCount++;

        this.name = name;

        setupNT(kp, ki, kd);
    }

    /**
     * Allocates a PIDController with the given constants for kp, ki, and kd.
     * Takes in a name for the NetworkTable
     *
     * @param kp     The proportional coefficient.
     * @param ki     The integral coefficient.
     * @param kd     The derivative coefficient.
     * @param period The period between controller updates in seconds.
     * @param name   The name to use in the NetworkTable
     * @throws IllegalArgumentException if kp &lt; 0
     * @throws IllegalArgumentException if ki &lt; 0
     * @throws IllegalArgumentException if kd &lt; 0
     * @throws IllegalArgumentException if period &lt;= 0
     */
    public NetworkedPID(double kp, double ki, double kd, double period, String name) {
        super(kp, ki, kd);
        instanceCount++;

        this.name = name;

        setupNT(kp, ki, kd);
    }

    private void setupNT(double kp, double ki, double kd) {
        NetworkTableInstance defaultNT = NetworkTableInstance.getDefault();

        this.kpTopic = defaultNT.getDoubleTopic("/NetworkedLib/" + name + "/kp");
        this.kiTopic = defaultNT.getDoubleTopic("/NetworkedLib/" + name + "/ki");
        this.kdTopic = defaultNT.getDoubleTopic("/NetworkedLib/" + name + "/kd");

        this.kpSub = kpTopic.subscribe(kp);
        this.kiSub = kiTopic.subscribe(ki);
        this.kdSub = kdTopic.subscribe(kd);

        this.kpPub = kpTopic.publish();
        this.kiPub = kiTopic.publish();
        this.kdPub = kdTopic.publish();

        kpPub.set(kp);
        kiPub.set(ki);
        kdPub.set(kd);
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @param setpoint    The new setpoint of the controller.
     * @return The next controller output.
     */
    public double calculate(double measurement, double setpoint) {
        UpdatePID();
        return super.calculate(measurement, setpoint);
    }

    /**
     * Updates the current PID with new values
     */
    public void UpdatePID() {
        this.setPID(kpSub.get(), kiSub.get(), kdSub.get());
    }
}
