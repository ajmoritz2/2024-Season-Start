package frc.robot.utils.maths;

import java.util.Arrays;

public class oneDimensionalLookup {

    public static final double interpLinear(double[] x, double[] y, double xi) throws IllegalArgumentException {

        if (x.length != y.length) {
            throw new IllegalArgumentException("X and Y must be the same length");
        }
        if (x.length == 1) {
            throw new IllegalArgumentException("X must contain more than one value");
        }
        double[] dx = new double[x.length - 1];
        double[] dy = new double[x.length - 1];
        double[] slope = new double[x.length - 1];
        double[] intercept = new double[x.length - 1];

        // Calculate the line equation (i.e. slope and intercept) between each point
        for (int i = 0; i < x.length - 1; i++) {
            dx[i] = x[i + 1] - x[i];
            if (dx[i] == 0) {
                throw new IllegalArgumentException("X must be montotonic. A duplicate " + "x-value was found");
            }
            if (dx[i] < 0) {
                throw new IllegalArgumentException("X must be sorted");
            }
            dy[i] = y[i + 1] - y[i];
            slope[i] = dy[i] / dx[i];
            intercept[i] = y[i] - x[i] * slope[i];
        }

        // Perform the interpolation here
        double yi = 0;
        if (xi > x[x.length - 1]) {
            yi = y[y.length - 1];
        } else if (xi < x[0]){
            yi = y[0];
        }
        else {
            int loc = Arrays.binarySearch(x, xi);
            if (loc < -1) {
                loc = -loc - 2;
                yi = slope[loc] * xi + intercept[loc];
            }
            else {
                yi = y[loc];
            }
        }
        
        return yi;
    }
}