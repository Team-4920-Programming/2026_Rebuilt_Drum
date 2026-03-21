package frc.robot.subsystems.DataHighway;

import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

/**
 * Lookup table for shooter parameters based on distance to the goal.
 *
 * Each entry maps a distance (meters) to:
 *   - Shooter RPM
 *   - Hood angle (degrees)
 *   - Time of flight (seconds)
 *
 * All values are linearly interpolated between entries using WPILib's
 * InterpolatingDoubleTreeMap.
 *
 * Reference: https://blog.eeshwark.com/blog/shooting-on-the-fly-pt2
 */
public class ShooterLookupTable {


  public record ShooterParams(double rpm,double hoodAngle, double timeOfFlight) implements Interpolatable<ShooterParams>{

    @Override
    public ShooterParams interpolate(ShooterParams other, double t){
      return new ShooterParams(rpm + (other.rpm - rpm) * t, hoodAngle + (other.hoodAngle - hoodAngle) * t, timeOfFlight + (other.timeOfFlight - timeOfFlight) * t );
    }

  }
  public final InterpolatingTreeMap<Double, ShooterParams> shooterTable = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolatable::interpolate);
  
  public ShooterLookupTable() {

        // shooterTable.put(distance,new ShooterParams(rpm, hoodAngle, timeofFlight));
    shooterTable.put(1.2,new ShooterParams(1325, 0.0, 0.8125));
    shooterTable.put(2.0,new ShooterParams(1400, 0.0, 0.9375));
    shooterTable.put(2.5,new ShooterParams(1550, 0.5, 1.0625));
    shooterTable.put(3.0,new ShooterParams(1500, 11.0, 1.0625));
    shooterTable.put(3.5,new ShooterParams(1700, 20.0, 1.0625));
    shooterTable.put(4.0,new ShooterParams(1700, 23.0, 1.375));
    shooterTable.put(5.0,new ShooterParams(1850, 22.0, 1.5625));
    // TODO: Fill in with real measured data from testing.
    // Format: addEntry(distanceMeters, rpm, hoodAngleDegrees, timeOfFlightSeconds)
    //
    // Measure these by:
    //   1. Place robot at known distance from goal
    //   2. Tune RPM and hood angle until shots consistently score
    //   3. Measure time of flight (high-speed camera or ball-exit to goal-entry sensors)
    //   4. Record all three values for each distance

    // addEntry(1.30, 2600, 74.5, 1.0);
    // addEntry(2.0, 3000, 74.5, 1.25);
    // addEntry(2.0, 2600, 62.8, 0.9375);//
    // addEntry(2.5, 3600, 74.5, 1.5625);
    // addEntry(2.5, 3000, 67.75, 1.125 );
    //     addEntry(3.0, 3600, 74.5, 1.4375);
    // addEntry(3.0, 3000, 61.75, 1.0 );
    // addEntry(2.5, 3200, 44, 0.6);
    // addEntry(3.0, 3600, 48, 0.7);
    // addEntry(3.5, 3900, 51, 0.8);
    // addEntry(4.0, 4200, 54, 0.9);
    // addEntry(4.5, 4500, 56, 1.0);
    // addEntry(5.0, 4800, 58, 1.1);
  }

}