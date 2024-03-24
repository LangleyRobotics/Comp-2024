package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterLookupTable {
    public static ShooterLookupTable instance;

    public static ShooterLookupTable getInstance() {
        if (instance == null) instance = new ShooterLookupTable();
        return instance;
    }

    private final InterpolatingDoubleTreeMap distanceToShooterRPM, distanceToPivotAngle;

    public ShooterLookupTable() {
        distanceToShooterRPM = new InterpolatingDoubleTreeMap();
        distanceToPivotAngle = new InterpolatingDoubleTreeMap();

        // TODO - may be percentage instead of RPM, check smartdashboard.
        // TABLE FORMAT: DISTANCE, SHOOTER RPM, PIVOT ANGLE (DEG) //
        double[][] lookupTable = {
                {1.65, 1, 176},
                {1.88, 1, 171},
                {2.05, 1, 163},
                {2.14, 1, 156},
                {2.45, 1, 154.3},
                {2.51, 1, 151.5},
                {3, 1, 147},
                {3.32, 1, 146.39},
                {3.68, 1, 145.2},
                {3.91, 1, 144.0},
                {4.22, 1, 144.0}
        };
        createLookupTable(lookupTable);
    }

    public ActionSetpoint get(double distanceToTarget) {
        return new ActionSetpoint(distanceToShooterRPM.get(distanceToTarget), distanceToPivotAngle.get(distanceToTarget));
    }

    private void createLookupTable(double[][] table) {
        for (double[] t : table) {
            Double distance = t[0];
            distanceToShooterRPM.put(distance, t[1]);
            distanceToPivotAngle.put(distance, t[2]);
        }
    }

    public static class ActionSetpoint {
        private final double shooterRPM, pivotAngle;

        public ActionSetpoint(double shooterRPM, double pivotAngle) {
            this.shooterRPM = shooterRPM;
            this.pivotAngle = pivotAngle;
        }

        public double getShooterRPM() {
            return shooterRPM;
        }

        public double getPivotAngle() {
            return pivotAngle;
        }
    }
}
