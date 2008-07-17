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
import java.util.List;

import simpull.SimpleSpring.SpringParticle;

public strictfp class SimpullCollection {
	
	protected List<Particle> particles = new ArrayList<Particle>();
	protected List<IConstraint> constraints = new ArrayList<IConstraint>();
	protected boolean hasParent;
	
	private SpringParticle tmpCollisionParticle;
	
	///////////////////////////////////////////////////////////////////////////
	// Collision variables
	private static Vector2f collisionNormal = new Vector2f();
	private static float collisionDepth = 0f;
	private static Collision collisionA = new Collision();
	private static Collision collisionB = new Collision();

	public List<Particle> getParticles() {
		return particles;
	}
	
	/**
	 * The Array of all AbstractConstraint instances added to the AbstractCollection
	 */	
	public List<IConstraint> getConstraints() {
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
	
	void integrate(float dt2) {
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
						spring.getCollisionParticle().updatePosition();
						detectCollisionDelegate(jParticle, spring.getCollisionParticle());
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
    	Vector2f mtd = new Vector2f(collisionNormal.x * collisionDepth, 
    			collisionNormal.y * collisionDepth);           
        float totalElasticity = collisionA.me.getElasticity() + collisionB.me.getElasticity();
        float sumInvMass = collisionA.me.getInvMass() + collisionB.me.getInvMass();
        
        // the total friction in a collision is combined but clamped to [0,1]
        float totalFriction = MathUtil.clamp(1 - (collisionA.me.getFriction() + collisionB.me.getFriction()), 0, 1);
        
        populateCollision(collisionA);
        populateCollision(collisionB);

        // calculate the coefficient of restitution as the normal component
        float teaim = (totalElasticity + 1) * collisionA.me.getInvMass();
        Vector2f bnaim = new Vector2f(collisionB.vNormal.x * teaim, collisionB.vNormal.y * teaim);
        float bimaim = collisionB.me.getInvMass() - totalElasticity * collisionA.me.getInvMass();
        Vector2f bnaim_plus_abimaim = new Vector2f(bnaim.x + collisionA.vNormal.x * bimaim, bnaim.y + collisionA.vNormal.y * bimaim);
        Vector2f vNormalA = bnaim_plus_abimaim.divEquals(sumInvMass);
        
        float tebim = (totalElasticity + 1) * collisionB.me.getInvMass();
        Vector2f anaim = new Vector2f(collisionA.vNormal.x * tebim, collisionA.vNormal.y * tebim);
        float aimbim = collisionA.me.getInvMass() - totalElasticity * collisionB.me.getInvMass();
        Vector2f anaim_plus_abimaim = new Vector2f(anaim.x + collisionA.vNormal.x * aimbim, anaim.y + collisionA.vNormal.y * aimbim);
        Vector2f vNormalB = anaim_plus_abimaim.divEquals(sumInvMass);
        
        // apply friction to the tangental component
        collisionA.vTangent.x *= totalFriction;
        collisionA.vTangent.y *= totalFriction;
        collisionB.vTangent.x *= totalFriction;
        collisionB.vTangent.y *= totalFriction;
        
        // scale the mtd by the ratio of the masses. heavier particles move less
        float aMassPct = collisionA.me.getInvMass() / sumInvMass;
        Vector2f mtdA = new Vector2f(mtd.x * aMassPct, mtd.y * aMassPct);     
        float bNMassPct = -collisionB.me.getInvMass() / sumInvMass;
        Vector2f mtdB = new Vector2f(mtd.x * bNMassPct, mtd.y * bNMassPct);     
        // add the tangental component to the normal component for the new velocity 
        vNormalA.x += collisionA.vTangent.x;
        vNormalA.y += collisionA.vTangent.y;
        vNormalB.x += collisionB.vTangent.x;
        vNormalB.y += collisionB.vTangent.y;

        // Make sure to pass a copy of the collision because these variables change all the time
        collisionA.me.respondToCollision(Collision.copy(collisionA), mtdA, vNormalA, collisionNormal, collisionDepth, -1);
        collisionB.me.respondToCollision(Collision.copy(collisionB), mtdB, vNormalB, collisionNormal, collisionDepth,  1);
    }
    
    /**
     * Calculate the collision components vNormal and vTangent and populate collisionToPopulate appropriately.
     * @param collisionToPopulate collision object with the following properties already populated (me & you)
     */
    private static final void populateCollision(Collision collisionToPopulate) {
		Vector2f velocity = collisionToPopulate.me.getVelocity();
		float vdotn = collisionNormal.x * velocity.x + collisionNormal.y * velocity.y;
		collisionToPopulate.vNormal = 
			new Vector2f(collisionNormal.x * vdotn, collisionNormal.y * vdotn);
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
		float s = 1 / (particleA.getMultisample() + 1); 
		float t = s;
		
		for (int i = 0; i <= particleA.getMultisample(); ++i) {
			particleA.samp.x = particleA.prevPosition.x + t * (particleA.position.x - particleA.prevPosition.x); 
			particleA.samp.y = particleA.prevPosition.y + t * (particleA.position.y - particleA.prevPosition.y);
			
			if (testCollisionDelegate(particleA, particleB)) {
				initiateCollisionResponse();
				return;
			}
			t += s;
		}
	}

	/**
	 * Tests two particles where both are of equal multisample rate
	 */		
	private static final void detectCollisionSampVsSamp(Particle particleA, Particle particleB) {
		if (detectCollisionNormVsNorm(particleA,particleB)) {
			return;
		}
		float s = 1 / (particleA.getMultisample() + 1); 
		float t = s;
		
		for (int i = 0; i <= particleA.getMultisample(); ++i) {
			particleA.samp.x = particleA.prevPosition.x + t * (particleA.position.x - particleA.prevPosition.x); 
			particleA.samp.y = particleA.prevPosition.y + t * (particleA.position.y - particleA.prevPosition.y);
			
			particleB.samp.x = particleB.prevPosition.x + t * (particleB.position.x - particleB.prevPosition.x); 
			particleB.samp.y = particleB.prevPosition.y + t * (particleB.position.y - particleB.prevPosition.y);
			
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
		collisionDepth = Float.POSITIVE_INFINITY;
		for (int i = 0; i < 2; ++i) {
			Vector2f axisA = obbA.axes[i];
		    float depthA = testIntervals(
		    		obbA.getProjection(axisA), obbB.getProjection(axisA));
		    if (depthA == 0) {
		    	return false;
		    }
		    Vector2f axisB = obbB.axes[i];
		    float depthB = testIntervals(
		    		obbA.getProjection(axisB), obbB.getProjection(axisB));
		    if (depthB == 0) {
		    	return false;
		    }
		    float absA = Math.abs(depthA);
		    float absB = Math.abs(depthB);
		    
		    if (absA < Math.abs(collisionDepth) || absB < Math.abs(collisionDepth)) {
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
		collisionDepth = Float.POSITIVE_INFINITY;
		float []depths = new float[2];
		
		// first go through the axes of the rectangle
		for (int i = 0; i < 2; ++i) {
			Vector2f boxAxis = obb.axes[i];
			float depth = testIntervals(
					obb.getProjection(boxAxis), circle.getProjection(boxAxis));
			if (depth == 0) return false;

			if (Math.abs(depth) < Math.abs(collisionDepth)) {
				collisionNormal = boxAxis;
				collisionDepth = depth;
			}
			depths[i] = depth;
		}	
		
		// determine if the circle's center is in a vertex region
		float r = circle.getRadius();
		if (Math.abs(depths[0]) < r && Math.abs(depths[1]) < r) {
			Vector2f vertex = findClosestVertexOnOBB(circle.samp, obb);

			// get the distance from the closest vertex on rect to circle center
			collisionNormal = new Vector2f(vertex.x - circle.samp.x, vertex.y - circle.samp.y);
			float mag = (float)Math.sqrt(collisionNormal.x * collisionNormal.x + collisionNormal.y * collisionNormal.y);
			collisionDepth = r - mag;

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
		float depthX = testIntervals(circleA.getIntervalX(), circleB.getIntervalX());
		if (depthX == 0) {
			return false;
		}
		float depthY = testIntervals(circleA.getIntervalY(), circleB.getIntervalY());
		if (depthY == 0) {
			return false;
		}
		collisionNormal = new Vector2f(circleA.samp.x - circleB.samp.x, circleA.samp.y - circleB.samp.y);
		float mag = (float)Math.sqrt(collisionNormal.x * collisionNormal.x + collisionNormal.y * collisionNormal.y);
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
	private static final float testIntervals(Interval intervalA, Interval intervalB) {
		if (intervalA.max < intervalB.min) {
			return 0;
		}
		if (intervalB.max < intervalA.min) {
			return 0;
		}
		float lenA = intervalB.max - intervalA.min;
		float lenB = intervalB.min - intervalA.max;
		
		return (Math.abs(lenA) < Math.abs(lenB)) ? lenA : lenB;
	}
	
	/** @return the location of the closest vertex on obb to point */
 	private static final Vector2f findClosestVertexOnOBB(Vector2f point, Rectangle obb) {
 		Vector2f d = new Vector2f(point.x - obb.samp.x, point.y - obb.samp.y);
 		Vector2f q = new Vector2f(obb.samp.x, obb.samp.y);
		for (int i = 0; i < 2; ++i) {
			float dist = d.x * obb.axes[i].x + d.y * obb.axes[i].y;
			if (dist >= 0) {
				dist = obb.extents[i];
			}
			else if (dist < 0) {
				dist = -obb.extents[i];
			}
			q.x += obb.axes[i].x * dist;
			q.y += obb.axes[i].y * dist;
		}
		return q;
	}

}