package frc.robot.subsystems.DataHighway;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.List;

import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
  public final List<Double> keyList = List.of(1.0,1.3,2.1,2.5,3.0,3.9,4.9);
  public final InterpolatingTreeMap<Double, ShooterParams> shooterTable = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolatable::interpolate);
  public final InterpolatingDoubleTreeMap inverseShooterTable = new InterpolatingDoubleTreeMap();
  public ShooterLookupTable() {

        // shooterTable.put(distance,new ShooterParams(rpm, hoodAngle, timeofFlight));
    shooterTable.put(1.0,new ShooterParams(2000, 0.0, 0.875));
    shooterTable.put(1.3,new ShooterParams(2200, 0, 0.75));
    shooterTable.put(2.1,new ShooterParams(2350, 4.5, 0.875));
    shooterTable.put(2.5,new ShooterParams(2400, 9.0, 0.9375));
    shooterTable.put(3.0,new ShooterParams(2600, 15.0, 0.9375));
    shooterTable.put(3.9,new ShooterParams(2950, 19.0, 1.375));
    shooterTable.put(4.9,new ShooterParams(3000, 28.0, 1.0625));
    shooterTable.put(8.7,new ShooterParams(3500, 41.0, 1.3125));

    setupInverseMap();
  }

  public void setupInverseMap(){
    for (var key : keyList){
      double distance = (double)key;
      inverseShooterTable.put(distance/(shooterTable.get(distance).timeOfFlight()), distance);
    }
  }

}