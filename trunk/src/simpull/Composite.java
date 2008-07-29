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
import java.util.Queue;

import simpull.events.Event;
import simpull.events.EventHandler;
import simpull.events.EventParticipator;
import simpull.events.sample.Events;
import simpull.events.sample.Events.CollisionEvent;
	
/**
 * The Composite class can contain Particles, and Constraints.
 * The most common use is to create a complex physics object.
 * The Composite can be rotated all as one around a single point. 
 * Composites can be added to a parent Group, along with Particles and Constraints.  
 * Members of a Composite are not checked for collision with one another, internally.
 * NOTE: All collision events are passed straight through to the center particle.
 * 			In fact, all {@link EventParticipator} method implementations delegate to the
 * 			center particle.
 */ 
public strictfp class Composite extends SimpullCollection implements IPhysicsObject, EventParticipator {
	
	/** This is used as a reference point, helpful in determining the current rotation. */
	protected Particle firstParticleAdded;
	protected final Particle centerParticle;
	
	private static final String UNABLE_TO_MODIFY_MESSAGE = "Cannot modify a Composite after it has been marked complete.";
	private static final String ALREADY_COMPLETE_MESSAGE = "Cannot mark a Composite as complete that is already marked complete.";
	private static final String CANNOT_ADD_INCOMPLETE_MESSAGE = "Cannot add a Composite that is not marked complete.";
	private static final String CANNOT_ADD_PARENTED_MESSAGE = "Cannot add a Composite that has already been added to another Composite.";
	private static final String NO_CONNECTION_TO_CENTER_MESSAGE = "Cannont mark a Composite as complete that does not have any connection to the center particle.";
	
	private List<Composite> composites = new ArrayList<Composite>();
	private Composite parentComposite;
	private Vector2f delta = new Vector2f();
	private boolean compositeCompleted;
	private float initialReferenceRotation;
	private float angularVelocity;
	/** When no coordinates are given, the center point will be calculated based on the layout of the particles added */
	private boolean calculateCenterPoint;
	private float mass;
	private boolean autoConnectParticlesToCenter;
	
	/**
	 * The center position of this object will be determined and set when all components are 
	 * added and {@link Composite#setCompositeCompleted(boolean)} is called.  
	 * @param isFixed determines if the center point is fixed position or not
	 * @param mass the total mass of the {@link Composite}, distributed among all {@link IPhysicsObject} added
	 * @param autoConnectParticlesToCenter true if you want to automatically have the center point connected
	 * 		to each particle added during {@link #setCompositeCompleted(simpull.Composite.Finisher...)}
	 * 		via a {@link SimpleSpring}, false will get no such auto addition of connectors to center, but beware
	 * 		that in order for a {@link Composite} to function as expected some connections to the center must be made
	 * 		manually before {@link #setCompositeCompleted(simpull.Composite.Finisher...)} is called.
	 */
	public Composite(boolean isFixed, float mass, boolean autoConnectParticlesToCenter) {
		centerParticle = new Particle(0, 0, isFixed, mass, 0f, 0f);
		this.mass = mass;
		this.autoConnectParticlesToCenter = autoConnectParticlesToCenter;
		calculateCenterPoint = true;
		centerParticle.isCollidable = false;
	}
	
	/**
	 * 
	 * @param isFixed determines if the center point is fixed position or not
	 * @param mass the total mass of the {@link Composite}, distributed among all {@link IPhysicsObject} added
	 * @param autoConnectParticlesToCenter true if you want to automatically have the center point connected
	 * 		to each particle added during {@link #setCompositeCompleted(simpull.Composite.Finisher...)}
	 * 		via a {@link SimpleSpring}, false will get no such auto addition of connectors to center, but beware
	 * 		that in order for a {@link Composite} to function as expected some connections to the center must be made
	 * 		manually before {@link #setCompositeCompleted(simpull.Composite.Finisher...)} is called.
	 * @param centerX the non-derived x value of the center particle of this composite
	 * @param centerY the non-derived y value of the center particle of this composite
	 */
	public Composite(boolean isFixed, float mass, boolean autoConnectParticlesToCenter, float centerX, float centerY) {
		centerParticle = new Particle(centerX, centerY, isFixed, mass, 0f, 0f);
		this.mass = mass;
		this.autoConnectParticlesToCenter = autoConnectParticlesToCenter;
		calculateCenterPoint = false;
		centerParticle.isCollidable = false;
	}
	
	/**
	 * After adding all {@link Particle} and {@link IConstraint} instances
	 * to this composite, it must be marked as complete to allow further processing.
	 * @param finishers a way for child classes to perform additional processing when the composite is complete.
	 * 			These are called before any internal processing takes place.
	 * @throws IllegalStateException if already marked as complete
	 */
	public final void setCompositeCompleted(Finisher... finishers) throws IllegalStateException {
		if (compositeCompleted) {
			throw new IllegalStateException(ALREADY_COMPLETE_MESSAGE);
		}
		for (Finisher finisher : finishers) {
			finisher.finish(this);
		}
		for (Particle particle : particles) {
			// Pass all collision events up from the elements to this top level particle.
			// TODO This should also include collidable constraints
			particle.eventAddHandler(Events.CollisionEvent.class.getName(), new EventHandler<Events.CollisionEvent>() {
				@Override
				public void handle(CollisionEvent event) {
					centerParticle.eventAttemptNotify(event);
				}
			});
			particle.eventAddHandler(Events.FirstCollisionEvent.class.getName(), new EventHandler<Events.FirstCollisionEvent>() {
				@Override
				public void handle(Events.FirstCollisionEvent event) {
					centerParticle.eventAttemptNotify(event);
				}
			});
		}
		setupCenter();
		compositeCompleted = true;
		initialReferenceRotation = getRelativeAngle(centerParticle.position, firstParticleAdded.position);
		setMass(mass);
		// TODO Does this need to take place for elasticity and friction too?
	}
	
	public void setAngularVelocity(float angularVelocity) {
		this.angularVelocity = angularVelocity;
	}
	
	public void setElasticity(float elasticity) {
		// TODO Distribute
	}

	public void setFriction(float friction) {
		// TODO Distribute
	}

	/**
	 * Distributes the mass passed throughout each particle in this collection.
	 * The center particle is assigned the exact value passed in.
	 * @param mass
	 */
	public void setMass(float mass) {
		// TODO maybe make a generic method for distributing a value and pass a visitor to know what is distributed
		float massSum = 0f;
		for (Particle particle : particles) {
			massSum += particle.getMass();
		}
		float centerMass = mass / (float)particles.size();
		float nonCenterMass = mass - centerMass;
		for (Particle particle : particles) {
			particle.setMass((particle.getMass() / massSum) * nonCenterMass);
		}
		centerParticle.setMass(centerMass); // centerParticle.setMass(0.1f);
	}
	
	public float getMass() {
		return mass;
	}

	public float getRotation() {
		// TODO make sure this is in the range 0 - 2pi
		return getRelativeAngle(centerParticle.position, firstParticleAdded.position) - initialReferenceRotation;
	}

	public void setRotation(float rotation) {
		rotateBy(rotation - getRotation(), centerParticle.position);
	}
	
	/** @return the fixed state of the Composite. */	
	public boolean getFixed() {
		for (Particle particle : particles) {
			if (!particle.isFixed) {
				return false;	
			}
		}
		return true;
	}

	/**
	 * @param isFixed the fixed state of the Composite. Setting this value to true or false will
	 * set all of this Composite's component particles to that value. Getting this 
	 * value will return false if any of the component particles are not fixed.
	 */	
	public void setFixed(boolean isFixed) {
		for (Particle particle : particles) {
			particle.isFixed = isFixed;	
		}
	}
	
	@Override
	public void init() {
		super.init();
		centerParticle.init();
	}

	@Override
	public void cleanup() {
		super.cleanup();
		centerParticle.cleanup();
	}

	@Override
	public void add(IConstraint... newConstraints) throws IllegalStateException {
		if (compositeCompleted) {
			throw new IllegalStateException(UNABLE_TO_MODIFY_MESSAGE);
		}
		super.add(newConstraints);
	}

	@Override
	public void add(Particle... newParticles) throws IllegalStateException {
		if (compositeCompleted) {
			throw new IllegalStateException(UNABLE_TO_MODIFY_MESSAGE);
		}
		super.add(newParticles);
		if (firstParticleAdded == null) {
			firstParticleAdded = newParticles[0];
		}
	}
	
	/**
	 * Add one/many {@link Composite} instances to this one.
	 * @param newComposites
	 * @throws IllegalStateException if this is already marked as complete, or if adding one that is not marked complete.
	 */
	public void add(Composite... newComposites) throws IllegalStateException {
		if (compositeCompleted) {
			throw new IllegalStateException(UNABLE_TO_MODIFY_MESSAGE);
		}
		for (Composite newComposite : newComposites) {
			if (!newComposite.compositeCompleted) {
				throw new IllegalStateException(CANNOT_ADD_INCOMPLETE_MESSAGE);
			}
			if (newComposite.getParentComposite() != null) { // if already belongs to a composite
				throw new IllegalStateException(CANNOT_ADD_PARENTED_MESSAGE);
			}
			newComposite.setParentComposite(this);
			composites.add(newComposite);
			for (Particle newParticle : newComposite.particles) {
				add(newParticle);
			}
			for (IConstraint newConstraint : newComposite.constraints) {
				add(newConstraint);
			}
		}
	}

	public void remove(IConstraint constraint) throws IllegalStateException {
		if (compositeCompleted) {
			throw new IllegalStateException(UNABLE_TO_MODIFY_MESSAGE);
		}
		remove(constraint);
	}

	public void remove(Particle particle) throws IllegalStateException {
		if (compositeCompleted) {
			throw new IllegalStateException(UNABLE_TO_MODIFY_MESSAGE);
		}
		remove(particle);
	}
	
	public void remove(Composite composite) throws IllegalStateException {
		if (compositeCompleted) {
			throw new IllegalStateException(UNABLE_TO_MODIFY_MESSAGE);
		}
		composites.remove(composite);
		composite.setParentComposite(null);
		// FIXME remove all the contents of the composite
	}
	
	public boolean isCompositeCompleted() {
		return compositeCompleted;
	}
	
	public Particle getCenterParticle() {
		return centerParticle;
	}
	
	@Override
	public void eventAddHandler(String eventName, EventHandler eventHandler) {
		// TODO Think about an implementation like this, or perhaps call to both (i.e. propogate upward)
//		if (parentComposite != null) {
//			parentComposite.eventAddHandler(eventName, eventHandler);
//		} else {
//			centerParticle.eventAddHandler(eventName, eventHandler);
//		}

		centerParticle.eventAddHandler(eventName, eventHandler);
	}

	@Override
	public boolean eventAttemptNotify(Event event) {
		return centerParticle.eventAttemptNotify(event);
	}

	@Override
	public boolean eventDispatchTo(String byName, Event event) {
		return centerParticle.eventDispatchTo(byName, event);
	}

	@Override
	public List<EventHandler> getEventHandlers(String eventName) {
		return centerParticle.getEventHandlers(eventName);
	}

	@Override
	public Queue<Event> getEventQueue() {
		return centerParticle.getEventQueue();
	}

	@Override
	void integrate(float dt2) {
		// because the parent composite will have all the particles/constraints added from here, nothing
		// good will come from calling integrate more than once
		if (parentComposite == null) {
			super.integrate(dt2);
		}
		for (Composite composite : composites) {
			composite.integrate(dt2);
		}
		centerParticle.update(dt2);
		// handle rotational velocity
		if (angularVelocity != 0f) {
			rotateBy(angularVelocity * dt2, centerParticle.position);
		}
	}
	
	protected void setParentComposite(Composite parentComposite) {
		this.parentComposite = parentComposite;
	}
	
	protected Composite getParentComposite() {
		return parentComposite;
	}
	
	/** @return the relative angle in radians from the center to the point */
	private float getRelativeAngle(Vector2f center, Vector2f point) {
		delta.x = point.x - center.x;
		delta.y = point.y - center.y;
		return (float)Math.atan2(delta.y, delta.x);
	}
	
	/** Rotates the Composite to an angle specified in radians, around a given center */
	private void rotateBy(float angleRadians, Vector2f center) {
		for (Particle particle : particles) {
			Vector2f diff = particle.getCenter();
			diff.x -= center.x;
			diff.y -= center.y;
			float radius = (float)Math.sqrt(diff.x * diff.x + diff.y * diff.y);
			float angle = getRelativeAngle(center, particle.getCenter()) + angleRadians;
			particle.position.x = (float)(Math.cos(angle) * radius) + center.x;
			particle.position.y = (float)(Math.sin(angle) * radius) + center.y;
		}
	}
	
	private void setupCenter() {
		if (calculateCenterPoint) {
			Vector2f centerPoint = calculateCenter();
			centerParticle.position.x = centerPoint.x;
			centerParticle.position.y = centerPoint.y;
		}
		// Just have to set this in order to keep the velocity at 0, b/c this sets the prevPosition...thus, keeps the velocity at 0
		centerParticle.setPosition(centerParticle.position.x, centerParticle.position.y);
		if (autoConnectParticlesToCenter) {
			for (Particle particle : particles) {
				SimpleSpring connectorToCenter = new SimpleSpring(centerParticle, particle, 1f, false, 1f, 1f, false);
				add(connectorToCenter);
			}
		} else {
			// Check that at least one constraint has a connection to the center point
			boolean hasConnectionToCenter = false;
			for (IConstraint constraint : constraints) {
				if (constraint.isConnectedTo(centerParticle)) {
					hasConnectionToCenter = true;
					break;
				}
			}
			if (!hasConnectionToCenter) {
				throw new IllegalStateException(NO_CONNECTION_TO_CENTER_MESSAGE);
			}
		}
	}
	
	private Vector2f calculateCenter() {
		Vector2f center = new Vector2f();
		
		float minX = Float.MAX_VALUE;
		float maxX = Float.MIN_VALUE;
		for (Particle particle : particles) {
			if (particle.position.x > maxX) {
				maxX = particle.position.x;
			}
			if (particle.position.x < minX) {
				minX = particle.position.x;
			}
		}
		center.x = ((maxX - minX) / 2f) + minX;

		float minY = Float.MAX_VALUE;
		float maxY = Float.MIN_VALUE;
		for (Particle particle : particles) {
			if (particle.position.y > maxY) {
				maxY = particle.position.y;
			}
			if (particle.position.y < minY) {
				minY = particle.position.y;
			}
		}
		center.y = ((maxY - minY) / 2f) + minY;
		
		return center;
	}
	
	public interface Finisher {
		public void finish(Composite composite);
	}
	
}