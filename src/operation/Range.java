package operation;

public class Range {

    public static double clip(double a, double min, double max) {
        if (a > max) {
            return max;
        }
        else if (a < min) {
            return min;
        }
        return a;
    }

}
