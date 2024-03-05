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
                {1.65, 1, 180},
                {1.88, 1, 174},
                {2.19, 1, 167},
                {2.39, 1, 162},
                {2.70, 1, 160},
                {3, 1, 155.35},
                {3.22, 1, 149.19},
                {3.531, 1, 149.36}
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
