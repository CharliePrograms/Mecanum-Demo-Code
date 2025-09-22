package org.firstinspires.ftc.teamcode.utils;

public class Utils {
    /**
     * Applies a deadband to the input value.
     *
     * @param input     The input value to apply the deadband to.
     * @param threshold The deadband threshold. Inputs with absolute values below this threshold will be set to zero.
     * @return The input value if its absolute value is greater than the threshold; otherwise, returns zero.
     */
    public static double applyDeadband(double input, double threshold) {
        return (Math.abs(input) > threshold) ? input : 0.0;
    }
}
