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

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;

import simpull.events.Event;
import simpull.events.EventHandler;
import simpull.events.EventManager;
import simpull.events.EventParticipator;
import simpull.events.sample.Events;
	
/**
 * Note that while a Particle can be rotated, it does not have angular
 * velocity. In other words, during collisions, the rotation is not altered, 
 * and the energy of the rotation is not applied to other colliding particles.
 */
public strictfp class Particle implements IPhysicsObject, EventParticipator {
	
	public Vector2f position;
	/**
	 * The fixed state of the particle. If the particle is fixed, it does not move in
	 * response to forces or collisions. Fixed particles are good for surfaces.
	 */
	public boolean isFixed;
	/**
	 * Sets the solidity of the item. If an item is not solid, then other items colliding
	 * with it will not respond to the collision. This property differs from 
	 * isCollidable in that you can still test for collision events if
	 * an item's isCollidable property is true and its isSolid
	 * property is false. 
	 * 
	 * The isCollidable property takes precedence over the isSolid
	 * property if isCollidable is set to false. That is, if isCollidable
	 * is false, it won't matter if isSolid is set to true or false.
	 * 
	 * NOTE: If you don't need to check for collision events, using isCollidable
	 * is more efficient. Always use isCollidable unless you need to
	 * handle collision events.
	 */	
	public boolean isSolid;
	/**
	 * Determines if the particle can collide with other particles or constraints.
	 * The default state is true.
	 */
	public boolean isCollidable;

	Vector2f prevPosition;
	Vector2f samp;
	Interval interval;
	
	private float rotation;
	private float mass;

	/** The forces acting on the particle */
	private Vector2f forces;
	private boolean hasFirstCollisionOccurred;
			
	private float elasticity;
	private float invMass;
	private float friction;
	
	private Vector2f center;
	private int multisample;
	
	private java.util.Queue<Event> eventQueue = new java.util.concurrent.ConcurrentLinkedQueue<Event>();
	private Map<String, List<EventHandler>> eventHandlersMap = new HashMap<String, List<EventHandler>>();
		
	public Particle (
			float x, 
			float y, 
			boolean isFixed, 
			float mass, 
			float elasticity,
			float friction) {
		interval = new Interval(0,0);
		position = new Vector2f(x, y);
		prevPosition = new Vector2f(x, y);
		samp = new Vector2f();
		this.isFixed = isFixed;
		forces = new Vector2f();
		isCollidable = true;
		hasFirstCollisionOccurred = false;
		setMass(mass);
		setElasticity(elasticity);
		setFriction(friction);
		rotation = 0;
		center = new Vector2f();
		multisample = 0;
		isSolid = true;
		EventManager.registerEventParticipator(this);
	}
	
	@Override
	public void init() {}
	
	@Override
	public void cleanup() {}

	public float getRotation() {
		return rotation;
	}

	public void setRotation(float rotation) {
		this.rotation = rotation;
	}
	
	public float getMass() {
		return mass;
	}
	
	/** @throws IllegalArgumentException when mass is <= 0 */
	public void setMass(float mass) {
		if (mass <= 0) {
			throw new IllegalArgumentException("mass may not be set <= 0"); 
		}
		this.mass = mass;
		invMass = 1 / mass;
	}	

	/**
	 * The elasticity of the particle. Standard values are between 0 and 1. 
	 * The higher the value, the greater the elasticity.
	 * 
	 * During collisions the elasticity values are combined. If one particle's
	 * elasticity is set to 0.4 and the other is set to 0.4 then the collision will
	 * be have a total elasticity of 0.8. The result will be the same if one particle
	 * has an elasticity of 0 and the other 0.8.
	 * 
	 * Setting the elasticity to greater than 1 (of a single particle, or in a combined
	 * collision) will cause particles to bounce with energy greater than naturally 
	 * possible.
	 */ 
	public float getElasticity() {
		return elasticity; 
	}
	
	public void setElasticity(float elasticity) {
		this.elasticity = elasticity;
	}
	
	/**
	 * Determines the number of intermediate position steps during each collision test.
	 * Setting this number higher on fast moving particles can prevent 'tunneling' 
	 * -- when a particle moves so fast it misses collision with another particle.
	 * 
	 * If two particles both have multisample levels greater than 0 then their 
	 * multisample levels must be equal to be tested correctly. For example, if one 
	 * particle has a multisample level of 4 and another has a multisample level of 5
	 * then the two particles will not be tested for multisampled collision. This is
	 * due to the unequal amount of steps in the collision test. 
	 * 
	 * Multisampling is currently buggy and should only be applied to particles that
	 * have high velocity. You can selectively apply multisampling by doing something
	 * like: 
	 * if (myparticle.velocity.magnitude() > 20) {
	 *     myparticle.multisample = 100;
	 * } else {
	 *     myparticle.multsample = 0;
	 * }
	 */ 
	public int getMultisample() {
		return multisample; 
	}
	
	public void setMultisample(int m) {
		multisample = m;
	}
	
	/**
	 * @return a Vector2f of the current location of the particle
	 */	
	public Vector2f getCenter() {
		center.x = getX();
		center.y = getY();
		return center;
	}
	
	/**
	 * The surface friction of the particle. Values must be in the range of 0 to 1.
	 * 
	 * 0 is no friction (slippery), 1 is full friction (sticky).
	 * 
	 * During collisions, the friction values are summed, but are clamped between 1 
	 * and 0. For example, If two particles have 0.7 as their surface friction, then
	 * the resulting friction between the two particles will be 1 (full friction).
	 *
	 * There is a bug in the current release where colliding non-fixed particles with
	 * friction greater than 0 will behave erratically. A workaround is to only set 
	 * the friction of fixed particles.
	 * 
	 */	
	public float getFriction() {
		return friction; 
	}

	/** @throws IllegalArgumentException if the friction is set less than zero or greater than 1 */
	public void setFriction(float friction) {
		if (friction < 0 || friction > 1)  {
			throw new IllegalArgumentException("Legal friction must be >= 0 and <=1");
		}
		this.friction = friction;
	}
	
	/**
	 * @return a copy of the position of the particle (i.e. changes to the object returned do not reflect).
	 * You can alter the position of a particle three ways: change its position, set
	 * its velocity, or apply a force to it. Setting the position of a non-fixed 
	 * particle is not the same as setting its fixed property to true. A particle held
	 * in place by its position will behave as if it's attached there by a 0 length
	 * spring constraint. 
	 */
	public Vector2f getPosition() {
		return new Vector2f(position.x, position.y);
	}
	
	/**
	 * Set the position to some arbitrary location.  This method of setting the position disallows 
	 * being able to determine the velocity by looking at the current and previous positions; therefore,
	 * the previous position is set to the values passed in x and y.
	 * @param x
	 * @param y
	 */
	public void setPosition(float x, float y) {
		position.x = x;
		position.y = y;
		
		prevPosition.x = x;
		prevPosition.y = y;
	}

	/** @return the x position of this particle */
	public float getX() {
		return position.x;
	}

	/** Set the x position of this particle and record the previous position for Verlet integration */
	public void setX(float x) {
		position.x = x;
		prevPosition.x = x;	
	}

	/** @return the y position of this particle */
	public float getY(){
		return position.y;
	}

	/** Set the y position of this particle and record the previous position for Verlet integration */
	public void setY(float y) {
		position.y = y;
		prevPosition.y = y;	
	}
	
	/**
	 * @return the velocity of the particle. If you need to change the motion of a particle, 
	 * you should either use this property, or one of the addForce methods. Generally,
	 * the addForce methods are best for slowly altering the motion. The velocity 
	 * property is good for instantaneously setting the velocity, e.g., for 
	 * projectiles.
	 */
	public Vector2f getVelocity() {
		return new Vector2f(position.x - prevPosition.x, position.y - prevPosition.y);
	}
	
	public void setVelocity(Vector2f velocity) {
		prevPosition.x = position.x - velocity.x;	
		prevPosition.y = position.y - velocity.y;	
	}
	
	/**
	 * Adds a force to the particle. Using this method to a force directly to the
	 * particle will only apply that force for a single Simpull.step() cycle. 
	 * 
	 * @param iForce An IForce object.
	 */ 
	public void addForce(IForce iForce) {
		Vector2f vForce = iForce.getValue(invMass);
		forces.x += vForce.x;
		forces.y += vForce.y;
	}
	
	/** This method integrates the particle.  Called during the Simpull.step() cycle. */
	public void update(float dt2) {
		if (isFixed) {
			return;
		}
		// accumulate global forces
		for (IForce iForce : Simpull.forces) {
			Vector2f force = iForce.getValue(invMass);
			forces.x += force.x;
			forces.y += force.y;
		}

		Vector2f positionBefore = new Vector2f(position.x, position.y);			
		forces.x *= dt2;
		forces.y *= dt2;
		Vector2f velocity = getVelocity();
		Vector2f netVelocity = new Vector2f(velocity.x + forces.x, velocity.y + forces.y);
		netVelocity.x *= Simpull.damping;
		netVelocity.y *= Simpull.damping;
		
		position.x += netVelocity.x;
		position.y += netVelocity.y;

		prevPosition.x = positionBefore.x;
		prevPosition.y = positionBefore.y;

		// clear all the forces out, the appropriate forces are added each step
		forces.x = 0;
		forces.y = 0;
	}
	
	/**
	 * Resets the collision state of the particle. This value is used in conjunction
	 * with the {@link Events.FirstCollisionEvent.} event.
	 */	
	public void resetFirstCollision() {
		hasFirstCollisionOccurred = false;
	}
	
	@Override
	@SuppressWarnings("unchecked")
	public boolean eventAttemptNotify(Event event) {
		if (eventHandlersMap.get(event.getName()) != null) {
			eventQueue.offer(event);
			return true;
		}
		return false;
	}
	
	@Override
	@SuppressWarnings("unchecked")
	public boolean eventDispatchTo(String byName, Event event) {
		return false;
	}
	
	/**
	 * Add an {@link EventHandler} to handle events of the type with the eventName passed.
	 * This implementation allows for multiple handlers for one event.
	 * They will be executed in the order in which they were added.
	 * @param eventHandler null to remove all handlers for the eventName
	 */
	@Override
	@SuppressWarnings("unchecked")
	public void eventAddHandler(String eventName, EventHandler eventHandler) {
		List<EventHandler> eventHandlers = eventHandlersMap.get(eventName);
		if (eventHandler == null) {
			if (eventHandlers != null && !eventHandlers.isEmpty()) {
				eventHandlers.clear();
			}
			return;
		}
		if (eventHandlers == null) {
			eventHandlers = new LinkedList<EventHandler>();
			eventHandlersMap.put(eventName, eventHandlers);
		}
		eventHandlers.add(eventHandler);
	}
	
	@Override
	@SuppressWarnings("unchecked")
	public List<EventHandler> getEventHandlers(String eventName) {
		return eventHandlersMap.get(eventName);
	}
	
	@Override
	@SuppressWarnings("unchecked")
	public Queue<Event> getEventQueue() {
		return eventQueue;
	}
	
	/** @return the actual forces Vector2f */
	Vector2f getForces() {
		return forces;
	}
	
	/**
	 * This method responds to a collision with the otherParticle
	 * in two ways:
	 * 1) physics update (position, velocity)
	 * 2) any custom collision response(s) (via EventHandler(s))
	 * @param collision
	 * @param mtd
	 * @param velocity
	 * @param n
	 * @param d
	 * @param o
	 */
	void respondToCollision(Collision collision, Vector2f mtd, Vector2f velocity, Vector2f n,
			float d, int o) {
		addCollisionEvent(collision);
		if (isFixed || !isSolid || !collision.you.isSolid) {
			return;
		}
		position.x = samp.x + mtd.x;
		position.y = samp.y + mtd.y;
		setVelocity(velocity);
	}
	
	void addCollisionEvent(Collision collision) {		
		eventAttemptNotify(new Events.CollisionEvent(collision));
		
		if (!hasFirstCollisionOccurred) {
			hasFirstCollisionOccurred = true;
			eventAttemptNotify(new Events.FirstCollisionEvent(collision));
		}
	}
	
	float getInvMass() {
		return isFixed ? 0 : invMass; 
	}
	
}	
