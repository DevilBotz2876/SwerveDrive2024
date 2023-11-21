package bhs.devilbotz.utils;

// PIDAutotuner class
public class PIDAutoTuner {

    // Constants
    private static final double M_PI = Math.PI;

    // Variables
    private double targetInputValue;
    private long loopInterval;
    private double minOutput;
    private double maxOutput;
    private ZNMode znMode;
    private int cycles;
    private int i;
    private boolean output;
    private double outputValue;
    private long t1;
    private long t2;
    private long microseconds;
    private long tHigh;
    private long tLow;
    private double max;
    private double min;
    private double pAverage;
    private double iAverage;
    private double dAverage;
    private double kp;
    private double ki;
    private double kd;

    // ZNMode enum
    public enum ZNMode {
        ZNModeBasicPID,
        ZNModeLessOvershoot,
        ZNModeNoOvershoot
    }

    // Constructor
    public PIDAutoTuner() {
    }

    // Set target input for tuning
    public void setTargetInputValue(double target) {
        targetInputValue = target;
    }

    // Set loop interval
    public void setLoopInterval(long interval) {
        loopInterval = interval;
    }

    // Set output range
    public void setOutputRange(double min, double max) {
        minOutput = min;
        maxOutput = max;
    }

    // Set Ziegler-Nichols tuning mode
    public void setZNMode(ZNMode zn) {
        znMode = zn;
    }

    // Set tuning cycles
    public void setTuningCycles(int tuneCycles) {
        cycles = tuneCycles;
    }

    // Initialize all variables before loop
    public void startTuningLoop(long us) {
        i = 0; // Cycle counter
        output = true; // Current output state
        outputValue = maxOutput;
        t1 = t2 = us; // Times used for calculating period
        microseconds = tHigh = tLow = 0; // More time variables
        max = -Double.MAX_VALUE; // Max input
        min = Double.MAX_VALUE; // Min input
        pAverage = iAverage = dAverage = 0;
    }

    // Run one cycle of the loop
    public double tunePID(double input, long us) {
        // Useful information on the algorithm used (Ziegler-Nichols method/Relay method)
        // http://www.processcontrolstuff.net/wp-content/uploads/2015/02/relay_autot-2.pdf
        // https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
        // https://www.cds.caltech.edu/~murray/courses/cds101/fa04/caltech/am04_ch8-3nov04.pdf

        // Basic explanation of how this works:
        // * Turn on the output of the PID controller to full power
        // * Wait for the output of the system being tuned to reach the target input value
        // and then turn the controller output off
        // * Wait for the output of the system being tuned to decrease below the target input
        // value and turn the controller output back on
        // * Do this a lot
        // * Calculate the ultimate gain using the amplitude of the controller output and
        // system output
        // * Use this and the period of oscillation to calculate PID gains using the
        // Ziegler-Nichols method

        // Calculate time delta
        //long prevMicroseconds = microseconds;
        microseconds = us;
        //float deltaT = microseconds - prevMicroseconds;

        // Calculate max and min
        max = Math.max(max, input);
        min = Math.min(min, input);

        // Output is on and input signal has risen to target
        if (output && input > targetInputValue) {
            // Turn output off, record current time as t1, calculate tHigh, and reset maximum
            output = false;
            outputValue = minOutput;
            t1 = us;
            tHigh = t1 - t2;
            max = targetInputValue;
        }

        // Output is off and input signal has dropped to target
        if (!output && input < targetInputValue) {
            // Turn output on, record current time as t2, calculate tLow

            output = true;
            outputValue = maxOutput;
            t2 = us;
            tLow = t2 - t1;

            // Calculate Ku (ultimate gain)
            // Formula given is Ku = 4d / πa
            // d is the amplitude of the output signal
            // a is the amplitude of the input signal
            double ku = (4.0 * ((maxOutput - minOutput) / 2.0)) / (M_PI * (max - min) / 2.0);

            // Calculate Tu (period of output oscillations)
            double tu = tLow + tHigh;

            // How gains are calculated
            // PID control algorithm needs Kp, Ki, and Kd
            // Ziegler-Nichols tuning method gives Kp, Ti, and Td
            //
            // Kp = 0.6Ku = Kc
            // Ti = 0.5Tu = Kc/Ki
            // Td = 0.125Tu = Kd/Kc
            //
            // Solving these equations for Kp, Ki, and Kd gives this:
            //
            // Kp = 0.6Ku
            // Ki = Kp / (0.5Tu)
            // Kd = 0.125 * Kp * Tu

            // Constants
            // https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
            double kpConstant, tiConstant, tdConstant;
            if (znMode == ZNMode.ZNModeBasicPID) {
                kpConstant = 0.6;
                tiConstant = 0.5;
                tdConstant = 0.125;
            } else if (znMode == ZNMode.ZNModeLessOvershoot) {
                kpConstant = 0.33;
                tiConstant = 0.5;
                tdConstant = 0.33;
            } else {
                // Default to No Overshoot mode as it is the safest
                kpConstant = 0.2;
                tiConstant = 0.5;
                tdConstant = 0.33;
            }

            // Calculate gains
            kp = kpConstant * ku;
            ki = (kp / (tiConstant * tu)) * loopInterval;
            kd = (tdConstant * kp * tu) / loopInterval;

            // Average all gains after the first two cycles
            if (i > 1) {
                pAverage += kp;
                iAverage += ki;
                dAverage += kd;
            }

            // Reset minimum
            min = targetInputValue;

            // Increment cycle count
            i ++;
        }

        // If loop is done, disable output and calculate averages
        if (i >= cycles) {
            output = false;
            outputValue = minOutput;
            kp = pAverage / (i - 1);
            ki = iAverage / (i - 1);
            kd = dAverage / (i - 1);
        }

        return outputValue;
    }

    // Get PID constants after tuning
    public double getKp() {
        return kp;
    }

    public double getKi() {
        return ki;
    }

    public double getKd() {
        return kd;
    }

    // Is the tuning loop finished?
    public boolean isFinished() {
        return (i >= cycles);
    }

    // Return number of tuning cycle
    public int getCycle() {
        return i;
    }
}
