package simpull;

import java.util.List;

import simpull.Composite.Finisher;

/**
 * Adds collidable constraints beteween the particles in the order they were added.
 * This will create a structured body, but in order to maintain that shape, 
 * it adds non-collidable constraints between each particle and all other particles.
 * @author Shaun Sheppard
 *
 */
public final class CustomBodyFinisher implements Finisher {
	
	public static final CustomBodyFinisher instance = new CustomBodyFinisher();

	@Override
	public void finish(Composite composite) {
		List<Particle> particles = composite.getParticles();
		int size = particles.size();
		// First go around the outsides of the body and connect the particles.
		// These spring will be collidable
		for (int i = 0; i < size; ++i) {
			int endIdx = (i + 1) == size ? 0 : (i + 1); // wrap around to the first particle when on the last one added.
			SimpleSpring connector = new SimpleSpring(particles.get(i), particles.get(endIdx), 1f, true, 1f, 1f, false);
			composite.add(connector);
		}
		
		// Now go on the inside and add supporting connectors (not collidable).
		if (size > 3) { // triangles do not need support connectors
			int connectorsToDo = size - 3;
			for (int iParticle = 0; iParticle < size; ++iParticle) {
				for (int iConnector = 0; iConnector < connectorsToDo; ++iConnector) {
					SimpleSpring connector = new SimpleSpring(particles.get(iParticle), 
							particles.get(iParticle + 2 + iConnector), 1f, false, 1f, 1f, false);
					composite.add(connector);
				}
				if (iParticle > 0) {
					if (--connectorsToDo <= 0) {
						break;
					}
				}
			}
		}
	}
	
	private CustomBodyFinisher() {}
	
}
