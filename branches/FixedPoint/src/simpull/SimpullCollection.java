/*
    Copyright (c) 2008, Shaun Curtis Sheppard
    All rights reserved.
    
    Redistribution and use in source and binary forms, with or without 
    modification, are permitted provided that the following conditions are met:

        * Redistributions of source code must retain the above copyright 
          notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright 
          notice, this list of conditions and the following disclaimer in the 
          documentation and/or other materials provided with the distribution.
        * Neither the name of Shaun Curtis Sheppard nor the names of its 
          contributors may be used to endorse or promote products derived from 
          this software without specific prior written permission.
    
    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
    POSSIBILITY OF SUCH DAMAGE.
*/

package simpull;
	
import java.util.ArrayList;

import simpull.SimpleSpring.SpringParticle;

public class SimpullCollection {
	
	protected ArrayList<Particle> particles = new ArrayList<Particle>();
	protected ArrayList<IConstraint> constraints = new ArrayList<IConstraint>();
	protected boolean hasParent;
	
	private SpringParticle tmpCollisionParticle;
	
	///////////////////////////////////////////////////////////////////////////
	// Collision variables
	private static Vector2f collisionNormal = new Vector2f();
	private static int collisionDepth = 0;
	private static Collision collisionA = new Collision();
	private static Collision collisionB = new Collision();

	public ArrayList<Particle> getParticles() {
		return particles;
	}
	
	/**
	 * The Array of all AbstractConstraint instances added to the AbstractCollection
	 */	
	public ArrayList<IConstraint> getConstraints() {
		return constraints;	
	}

	/**
	 * Adds one or more {@link Particle} instances.
	 * @param particles The particle(s) to be added.
	 */
	public void add(Particle... newParticles) {
		for (Particle particle : newParticles) {
			particles.add(particle);
			if (hasParent) {
				particle.init();
			}
		}
	}
	
	/**
	 * Removes a {@link Particle}.
	 * @param particle The particle to be removed.
	 */
	public void remove(Particle particle) {
		particles.remove(particle);
		particle.cleanup();
	}
	
	/**
	 * Adds one or more {@link IConstraint} instances.
	 * @param constraint The constraint to be added.
	 */
	public void add(IConstraint... newConstraints) {
		for (IConstraint constraint : newConstraints) {
			constraints.add(constraint);
			if (hasParent) {
				constraint.init();
			}
		}
	}

	/**
	 * Removes a {@link IConstraint}.
	 * @param constraint The constraint to be removed.
	 */
	public void remove(IConstraint constraint) {
		constraints.remove(constraint);
		constraint.cleanup();
	}
	
	/**
	 * Calls every member's init method.
	 */
	public void init() {
		for (Particle particle : particles) {
			particle.init();	
		}
		for (IConstraint constraint : constraints) {
			constraint.init();
		}
	}
	
	/**
	 * Calls every member's cleanup method.
	 * Automatically called when this is removed from its parent.
	 */
	public void cleanup() {
		for (Particle particle : particles) {
			particle.cleanup();	
		}
		for (IConstraint constraint : constraints) {
			constraint.cleanup();
		}
	}
			
	boolean getHasParent() {
		return hasParent;
	}	

	void setHasParent(boolean hasParent) {
		this.hasParent = hasParent;
	}	
	
	void integrate(int dt2) {
		for (Particle particle : particles) {
			particle.update(dt2);	
		}
	}		
	
	void satisfyConstraints() {
		for (IConstraint constraint : constraints) {
			constraint.resolve();
		}
	}			
	
	void checkInternalCollisions() {
		int numParticles = particles.size();
		for (int j = 0; j < numParticles; ++j) {
			Particle jParticle = particles.get(j);
			if (!jParticle.isCollidable) {
				continue;
			}
			// check against every other particle in this Collection
			for (int i = j + 1; i < numParticles; ++i) {
				Particle iParticle = particles.get(i);
				if (iParticle.isCollidable) {
					detectCollisionDelegate(jParticle, iParticle);
				}
			}
			
			// check against every other constraint in this Collection
			for (IConstraint constraint : constraints) {
				if (constraint instanceof SimpleSpring) { // TODO This if was not here, so are we now missing something???
					SimpleSpring spring = (SimpleSpring)constraint;
					if (spring.isCollidable() && !spring.isConnectedTo(jParticle)) {
						tmpCollisionParticle = spring.getCollisionParticle();
						tmpCollisionParticle.updatePosition();
						detectCollisionDelegate(jParticle, tmpCollisionParticle);
					}
				}
			}
		}
	}

	void checkCollisionsVsCollection(SimpullCollection otherCollection) {
		for (Particle myParticle : particles) {
			if (!myParticle.isCollidable) {
				continue;
			}
			// check against every particle in the other collection
			for (Particle otherParticle : otherCollection.particles) {
				if (otherParticle.isCollidable) {
					detectCollisionDelegate(myParticle, otherParticle);
				}
			}
			// check against every constraint in the other collection
			for (IConstraint otherConstraint : otherCollection.constraints) {
				if (otherConstraint instanceof SimpleSpring) { // TODO This if was not here, so are we now missing something???
					SimpleSpring theirSpring = (SimpleSpring)otherConstraint;
					if (theirSpring.isCollidable() && !theirSpring.isConnectedTo(myParticle)) {
						tmpCollisionParticle = theirSpring.getCollisionParticle();
						tmpCollisionParticle.updatePosition();
						detectCollisionDelegate(myParticle, tmpCollisionParticle);
					}
				}
			}
		}
		
		for (IConstraint myConstraint : constraints) {
			if (myConstraint instanceof SimpleSpring) { // TODO This if was not here, so are we now missing something???
				SimpleSpring mySpring = (SimpleSpring)myConstraint;
				if (!mySpring.isCollidable()) {
					continue;
				}
				// check against every particle in the other collection
				for (Particle otherParticle : otherCollection.particles) {
					if (otherParticle.isCollidable && !mySpring.isConnectedTo(otherParticle)) {
						tmpCollisionParticle = mySpring.getCollisionParticle();
						tmpCollisionParticle.updatePosition();
						detectCollisionDelegate(otherParticle, tmpCollisionParticle);
					}
				}
			}
		}
	}			

	/**
	 * The main collision detection method.  It delegates to appropriate detection method
	 * based on particle multisampling values.
	 */	
	private static final void detectCollisionDelegate(Particle particleA, Particle particleB) {
		if (particleA.isFixed && particleB.isFixed) {
			return;
		}
		if (particleA.getMultisample() == 0 
				&& particleB.getMultisample() == 0) {
			detectCollisionNormVsNorm(particleA, particleB);
		} else if (particleA.getMultisample() > 0 
				&& particleB.getMultisample() == 0) {
			detectCollisionSampVsNorm(particleA, particleB);
		} else if (particleB.getMultisample() > 0 
				&& particleA.getMultisample() == 0) {
			detectCollisionSampVsNorm(particleB, particleA);
		} else if (particleA.getMultisample() == particleB.getMultisample()) {
			detectCollisionSampVsSamp(particleA, particleB);
		} else {
			detectCollisionNormVsNorm(particleA, particleB);
		}
	}
	
	/**
	 * Start the process to respond to the collision with the involved particles.
	 * Once here, we know there has been a collision, and we
	 * ASSUME/EXPECT each of the following variables are set:
	 * 	-collisionA.me/you
	 * 	-collisionB.me/you
	 *  -collisionDepth
	 *  -collisionNormal
	 */
    private static final void initiateCollisionResponse() {
    	Vector2f mtd = new Vector2f((int) (((long) collisionNormal.x * collisionDepth) >> FP.FRACTION_BITS), 
    			(int) (((long) collisionNormal.y * collisionDepth) >> FP.FRACTION_BITS));
// above replaces    	Vector2f mtd = new Vector2f(collisionNormal.x * collisionDepth, 
//    			collisionNormal.y * collisionDepth);
    	
        int invMassA = collisionA.me.getInvMass(); // used alot below, avoid method calls and calc
        int invMassB = collisionB.me.getInvMass(); // used alot below, avoid method calls and calc
        
    	int totalElasticity = collisionA.me.getElasticity() + collisionB.me.getElasticity();
        int sumInvMass = invMassA + invMassB;
        
        // the total friction in a collision is combined but clamped to [0,1]
        int totalFriction = FP.clamp(FP.ONE - (collisionA.me.getFriction() + collisionB.me.getFriction()), 0, FP.ONE);
        
        populateCollision(collisionA);
        populateCollision(collisionB);

        // calculate the coefficient of restitution as the normal component
        int teaim = (int) (((long) (totalElasticity + FP.ONE) * invMassA) >> FP.FRACTION_BITS);
        // above replaces float teaim = (totalElasticity + 1) * invMassA;
        
        Vector2f bnaim = new Vector2f((int) (((long) collisionB.vNormal.x * teaim) >> FP.FRACTION_BITS), 
        		(int) (((long) collisionB.vNormal.y * teaim) >> FP.FRACTION_BITS));
        // above replaces Vector2f bnaim = new Vector2f(collisionB.vNormal.x * teaim, collisionB.vNormal.y * teaim);
        
        int bimaim = invMassB - ((int) (((long) totalElasticity * invMassA) >> FP.FRACTION_BITS));
        // above replaces float bimaim = invMassB - totalElasticity * invMassA;
        
        Vector2f bnaim_plus_abimaim = new Vector2f(bnaim.x + ((int) (((long) collisionA.vNormal.x * bimaim) >> FP.FRACTION_BITS)),
        		bnaim.y + ((int) (((long) collisionA.vNormal.y * bimaim) >> FP.FRACTION_BITS)));
        // above replaces Vector2f bnaim_plus_abimaim = new Vector2f(bnaim.x + collisionA.vNormal.x * bimaim, bnaim.y + collisionA.vNormal.y * bimaim);
        
        Vector2f vNormalA = bnaim_plus_abimaim.divEquals(sumInvMass);
        
        int tebim = (int) (((long) (totalElasticity + FP.ONE) * invMassB) >> FP.FRACTION_BITS);
        // above replaces float tebim = (totalElasticity + 1) * invMassB;
        
        Vector2f anaim = new Vector2f((int) (((long) collisionA.vNormal.x * tebim) >> FP.FRACTION_BITS),
        		(int) (((long) collisionA.vNormal.y * tebim) >> FP.FRACTION_BITS));
        // above replaces Vector2f anaim = new Vector2f(collisionA.vNormal.x * tebim, collisionA.vNormal.y * tebim);
        
        int aimbim = invMassA - ((int) (((long) totalElasticity * invMassB) >> FP.FRACTION_BITS));
        // above replaces float aimbim = invMassA - totalElasticity * invMassB;
        
        Vector2f anaim_plus_abimaim = new Vector2f(anaim.x + ((int) (((long) collisionA.vNormal.x * aimbim) >> FP.FRACTION_BITS)), 
        		anaim.y + ((int) (((long) collisionA.vNormal.y * aimbim) >> FP.FRACTION_BITS)));
        // above replaces Vector2f anaim_plus_abimaim = new Vector2f(anaim.x + collisionA.vNormal.x * aimbim, anaim.y + collisionA.vNormal.y * aimbim);
        
        Vector2f vNormalB = anaim_plus_abimaim.divEquals(sumInvMass);
        
        // apply friction to the tangental component
        collisionA.vTangent.x = (int) (((long) collisionA.vTangent.x * totalFriction) >> FP.FRACTION_BITS);
        // above replaces collisionA.vTangent.x *= totalFriction;
        
        collisionA.vTangent.y = (int) (((long) collisionA.vTangent.y * totalFriction) >> FP.FRACTION_BITS);
        // above replaces collisionA.vTangent.y *= totalFriction;
        
        collisionB.vTangent.x = (int) (((long) collisionB.vTangent.x * totalFriction) >> FP.FRACTION_BITS);
        // above replaces collisionB.vTangent.x *= totalFriction;
        
        collisionB.vTangent.y = (int) (((long) collisionB.vTangent.y * totalFriction) >> FP.FRACTION_BITS);
        // above replaces collisionB.vTangent.y *= totalFriction;
        
        // scale the mtd by the ratio of the masses. heavier particles move less
        int aMassPct = (int) (((long) invMassA << FP.FRACTION_BITS) / sumInvMass);
        // above replaces float aMassPct = invMassA / sumInvMass;
        
        Vector2f mtdA = new Vector2f((int) (((long) mtd.x * aMassPct) >> FP.FRACTION_BITS), 
        		(int) (((long) mtd.y * aMassPct) >> FP.FRACTION_BITS));
        // above replaces Vector2f mtdA = new Vector2f(mtd.x * aMassPct, mtd.y * aMassPct);     
        
        int bNMassPct = (int) (((long) -invMassB << FP.FRACTION_BITS) / sumInvMass);
        // above replaces float bNMassPct = -invMassB / sumInvMass;
        
        Vector2f mtdB = new Vector2f((int) (((long) mtd.x * bNMassPct) >> FP.FRACTION_BITS), 
        		(int) (((long) mtd.y * bNMassPct) >> FP.FRACTION_BITS));
        // above replaces Vector2f mtdB = new Vector2f(mtd.x * bNMassPct, mtd.y * bNMassPct);     
        
        // add the tangental component to the normal component for the new velocity 
        vNormalA.x += collisionA.vTangent.x;
        vNormalA.y += collisionA.vTangent.y;
        vNormalB.x += collisionB.vTangent.x;
        vNormalB.y += collisionB.vTangent.y;

        // Make sure to pass a copy of the collision because these variables change all the time
        collisionA.me.respondToCollision(Collision.copy(collisionA), mtdA, vNormalA, collisionNormal, collisionDepth, -FP.ONE);
        collisionB.me.respondToCollision(Collision.copy(collisionB), mtdB, vNormalB, collisionNormal, collisionDepth,  FP.ONE);
    }
    
    /**
     * Calculate the collision components vNormal and vTangent and populate collisionToPopulate appropriately.
     * @param collisionToPopulate collision object with the following properties already populated (me & you)
     */
    private static final void populateCollision(Collision collisionToPopulate) {
		Vector2f velocity = collisionToPopulate.me.getVelocity();
		
		int vdotn = (int) (((long) collisionNormal.x * velocity.x) >> FP.FRACTION_BITS) + (int) (((long) collisionNormal.y * velocity.y) >> FP.FRACTION_BITS);
		// above replaces float vdotn = collisionNormal.x * velocity.x + collisionNormal.y * velocity.y;
		
		collisionToPopulate.vNormal = 
			new Vector2f((int) (((long) collisionNormal.x * vdotn) >> FP.FRACTION_BITS), (int) (((long) collisionNormal.y * vdotn) >> FP.FRACTION_BITS));
// above replaces		collisionToPopulate.vNormal = 
//			new Vector2f(collisionNormal.x * vdotn, collisionNormal.y * vdotn);
		
		collisionToPopulate.vTangent = 
			new Vector2f(velocity.x - collisionToPopulate.vNormal.x, velocity.y - collisionToPopulate.vNormal.y);	
    }

    /** default test for two non-multisampled particles */
	private static final boolean detectCollisionNormVsNorm(Particle particleA, Particle particleB) {
		particleA.samp.x = particleA.position.x;
		particleA.samp.y = particleA.position.y;

		particleB.samp.x = particleB.position.x;
		particleB.samp.y = particleB.position.y;
		if (testCollisionDelegate(particleA, particleB)) {
			initiateCollisionResponse();
			return true;
		}
		return false;
	}
	
	/**
	 * Tests two particles where one is multisampled and the other is not. Let objectA
	 * be the multisampled particle.
	 */
	private static final void detectCollisionSampVsNorm(Particle particleA, Particle particleB) {
		if (detectCollisionNormVsNorm(particleA,particleB)) {
			return;
		}
		
		int s = FP.ONE / (particleA.getMultisample() + 1);
		// above replaces float s = 1 / (particleA.getMultisample() + 1); 
		
		int t = s;
		
		for (int i = 0; i <= particleA.getMultisample(); ++i) {
			particleA.samp.x = particleA.prevPosition.x + ((int) (((long) t * (particleA.position.x - particleA.prevPosition.x)) >> FP.FRACTION_BITS));
			// above replaces particleA.samp.x = particleA.prevPosition.x + t * (particleA.position.x - particleA.prevPosition.x); 

			particleA.samp.y = particleA.prevPosition.y + ((int) (((long) t * (particleA.position.y - particleA.prevPosition.y)) >> FP.FRACTION_BITS));
			// above replaces particleA.samp.y = particleA.prevPosition.y + t * (particleA.position.y - particleA.prevPosition.y);
			
			if (testCollisionDelegate(particleA, particleB)) {
				initiateCollisionResponse();
				return;
			}
			t += s;
		}
	}

	/** Tests two particles where both are of equal multisample rate */		
	private static final void detectCollisionSampVsSamp(Particle particleA, Particle particleB) {
		if (detectCollisionNormVsNorm(particleA, particleB)) {
			return;
		}
		
		int s = FP.ONE / (particleA.getMultisample() + 1);
		// above replaces float s = 1 / (particleA.getMultisample() + 1); 
		
		int t = s;
		
		for (int i = 0; i <= particleA.getMultisample(); ++i) {
			particleA.samp.x = particleA.prevPosition.x + ((int) (((long) t * (particleA.position.x - particleA.prevPosition.x)) >> FP.FRACTION_BITS));
			// above replaces particleA.samp.x = particleA.prevPosition.x + t * (particleA.position.x - particleA.prevPosition.x); 

			particleA.samp.y = particleA.prevPosition.y + ((int) (((long) t * (particleA.position.y - particleA.prevPosition.y)) >> FP.FRACTION_BITS));
			// above replaces particleA.samp.y = particleA.prevPosition.y + t * (particleA.position.y - particleA.prevPosition.y);
			
			particleB.samp.x = particleB.prevPosition.x + ((int) (((long) t * (particleB.position.x - particleB.prevPosition.x)) >> FP.FRACTION_BITS));
			// above replaces particleB.samp.x = particleB.prevPosition.x + t * (particleB.position.x - particleB.prevPosition.x); 

			particleB.samp.y = particleB.prevPosition.y + ((int) (((long) t * (particleB.position.y - particleB.prevPosition.y)) >> FP.FRACTION_BITS));
			// above replaces particleB.samp.y = particleB.prevPosition.y + t * (particleB.position.y - particleB.prevPosition.y);
			
			if (testCollisionDelegate(particleA, particleB)) {
				initiateCollisionResponse();
				return;	
			} 
			t += s;
		}
	}
	
	/**
	 * This is a delegate method that ends up calling the appropriate
	 * test for collision method for the types of physics objects passed in.
	 */	
	private static final boolean testCollisionDelegate(Particle particleA, Particle particleB) {	
		if (particleA instanceof Rectangle
				&& particleB instanceof Rectangle) {
			return testCollisionOBBvsOBB((Rectangle)particleA, (Rectangle)particleB);
		} else if (particleA instanceof Circle 
				&& particleB instanceof Circle) {
			return testCollisionCirclevsCircle((Circle)particleA, (Circle)particleB);
		} else if (particleA instanceof Rectangle 
				&& particleB instanceof Circle) {
			return testCollisionOBBvsCircle((Rectangle)particleA, (Circle)particleB);
		} else if (particleA instanceof Circle 
				&& particleB instanceof Rectangle)  {
			return testCollisionOBBvsCircle((Rectangle)particleB, (Circle)particleA);
		}
		return false;
	}

	/**
	 * Tests the collision between two Rectangles (aka OBBs). If there is a 
	 * collision it determines its axis and depth, and then passes it off
	 * for handling.
	 */
	private static final boolean testCollisionOBBvsOBB(Rectangle obbA, Rectangle obbB) {
		collisionDepth = FP.MAX_VALUE;
		for (int i = 0; i < 2; ++i) {
			Vector2f axisA = obbA.axes[i];
		    int depthA = testIntervals(
		    		obbA.getProjection(axisA), obbB.getProjection(axisA));
		    if (depthA == 0) {
		    	return false;
		    }
		    Vector2f axisB = obbB.axes[i];
		    int depthB = testIntervals(
		    		obbA.getProjection(axisB), obbB.getProjection(axisB));
		    if (depthB == 0) {
		    	return false;
		    }
		    int absA = FP.abs(depthA);
		    int absB = FP.abs(depthB);
		    
		    if (absA < FP.abs(collisionDepth) || absB < FP.abs(collisionDepth)) {
		    	boolean altb = absA < absB;
		    	collisionNormal = altb ? axisA : axisB;
		    	collisionDepth = altb ? depthA : depthB;
		    }
		}
		collisionA.me = obbA;
		collisionA.you = obbB;

		collisionB.me = obbB;
		collisionB.you = obbA;
		return true;
	}		

	/**
	 * Tests the collision between a {@link Rectangle} (aka an OBB) and a 
	 * {@link Circle}. If there is a collision it determines its axis and depth, and 
	 * then passes it off to the CollisionResolver.
	 */
	private static final boolean testCollisionOBBvsCircle(Rectangle obb, Circle circle) {
		collisionDepth = FP.MAX_VALUE;
		int []depths = new int[2];
		
		// first go through the axes of the rectangle
		for (int i = 0; i < 2; ++i) {
			Vector2f boxAxis = obb.axes[i];
			int depth = testIntervals(
					obb.getProjection(boxAxis), circle.getProjection(boxAxis));
			if (depth == 0) {
				return false;
			}
			if (FP.abs(depth) < FP.abs(collisionDepth)) {
				collisionNormal = boxAxis;
				collisionDepth = depth;
			}
			depths[i] = depth;
		}	
		
		// determine if the circle's center is in a vertex region
		int radius = circle.getRadius();
		if (FP.abs(depths[0]) < radius && FP.abs(depths[1]) < radius) {
			Vector2f vertex = findClosestVertexOnOBB(circle.samp, obb);

			// get the distance from the closest vertex on rect to circle center
			collisionNormal = new Vector2f(vertex.x - circle.samp.x, vertex.y - circle.samp.y);
			
			int mag = FP.sqrt((int) (((long) collisionNormal.x * collisionNormal.x) >> FP.FRACTION_BITS) + (int) (((long) collisionNormal.y * collisionNormal.y) >> FP.FRACTION_BITS));
			// above replaces float mag = (float)Math.sqrt(collisionNormal.x * collisionNormal.x + collisionNormal.y * collisionNormal.y);
			
			collisionDepth = radius - mag;

			if (collisionDepth > 0) {
				// there is a collision in one of the vertex regions
				collisionNormal.divEquals(mag);
			} else {
				// ra is in vertex region, but is not colliding
				return false;
			}
		}
		collisionA.me = obb;
		collisionA.you = circle;

		collisionB.me = circle;
		collisionB.you = obb;
		
		return true;
	}

	/**
	 * Tests the collision between two {@link Circle}. If there is a collision it 
	 * determines its axis and depth.
	 */	
	private static final boolean testCollisionCirclevsCircle(Circle circleA, Circle circleB) {
		int depthX = testIntervals(circleA.getIntervalX(), circleB.getIntervalX());
		if (depthX == 0) {
			return false;
		}
		int depthY = testIntervals(circleA.getIntervalY(), circleB.getIntervalY());
		if (depthY == 0) {
			return false;
		}
		collisionNormal = new Vector2f(circleA.samp.x - circleB.samp.x, circleA.samp.y - circleB.samp.y);
		
		int mag = FP.sqrt((int) (((long) collisionNormal.x * collisionNormal.x) >> FP.FRACTION_BITS) + (int) (((long) collisionNormal.y * collisionNormal.y) >> FP.FRACTION_BITS));
		// above replaces float mag = (float)Math.sqrt(collisionNormal.x * collisionNormal.x + collisionNormal.y * collisionNormal.y);
		
		collisionDepth = (circleA.getRadius() + circleB.getRadius()) - mag;
		
		if (collisionDepth > 0) {
			collisionNormal.divEquals(mag);
			collisionA.me = circleA;
			collisionA.you = circleB;

			collisionB.me = circleB;
			collisionB.you = circleA;
			return true;
		}
		return false;
	}

	/** @return 0 if intervals do not overlap,  smallest depth if they do. */
	private static final int testIntervals(Interval intervalA, Interval intervalB) {
		if (intervalA.max < intervalB.min) {
			return 0;
		}
		if (intervalB.max < intervalA.min) {
			return 0;
		}
		int lenA = intervalB.max - intervalA.min;
		int lenB = intervalB.min - intervalA.max;
		
		return (FP.abs(lenA) < FP.abs(lenB)) ? lenA : lenB;
	}
	
	/** @return the location of the closest vertex on obb to point */
 	private static final Vector2f findClosestVertexOnOBB(Vector2f point, Rectangle obb) {
 		Vector2f d = new Vector2f(point.x - obb.samp.x, point.y - obb.samp.y);
 		Vector2f q = new Vector2f(obb.samp.x, obb.samp.y);
		for (int i = 0; i < 2; ++i) {
			int dist = (int) (((long) d.x * obb.axes[i].x) >> FP.FRACTION_BITS) 
					 + (int) (((long) d.y * obb.axes[i].y) >> FP.FRACTION_BITS);
			// above replaces float dist = d.x * obb.axes[i].x + d.y * obb.axes[i].y;
			
			if (dist >= 0) {
				dist = obb.extents[i];
			} else if (dist < 0) {
				dist = -obb.extents[i];
			}
			
			q.x += (int) (((long) obb.axes[i].x * dist) >> FP.FRACTION_BITS);
			// above replaces q.x += obb.axes[i].x * dist;
			
			q.y += (int) (((long) obb.axes[i].y * dist) >> FP.FRACTION_BITS);
			// above replaces q.y += obb.axes[i].y * dist;
		}
		return q;
	}

}