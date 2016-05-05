The name is a play on words - **SIM**_ulation_ (of the physical world) **PULL** (a physical action, described through physics).  It also speaks to a primary goal of the engine - _simple_ to use.

  * A simple interface
  * Verlet integration of particles (fixed & non-fixed) in system
  * Joint/Constraints between particles in system (spring/angle)
  * Rectangle (w/ rotation) and Circle (w/ angular velocity) shaped particles
  * Composites (to create custom physics objects)
  * Collision detection & response (simulated physics response)
  * Extensible event system (allows for custom collision response + more)
  * Single threaded (probably not safe to use in multi-threaded apps, needs testing)

The most suitable use is likely video game physics simulations.  This is because the accuracy Simpull provides is not likely to meet the needs of hardcore simulations.

**See it in action**: http://simpull2core.sourceforge.net/

---

J2SE 1.4.2 is in its Java Technology End of Life (EOL) transition period. The EOL transition period began Dec, 11 2006 and will complete October 30th, 2008, when J2SE 1.4.2 will have reached its End of Service Life (EOSL).  Therefore, Simpull requires Java 1.5.  This allows for a clean API and a lesser burden of source code maintenance.