import pymunk
from pymunk import Vec2d
import numpy as np
# from ..world.RectangularWorld import RectangularWorld
# from ..agent.Agent import Agent
from ..world.objects.Wall import Wall
# from ..world.objects.StaticObject import StaticObject, StaticObjectConfig
from pymunk import Shape

def copyCoords(dest, src):
    coords = [0, 0]
    if isinstance(src, Vec2d):
        coords[0] = src.x
        coords[1] = src.y
    elif isinstance(src, np.ndarray):
        coords[0] = float(src[0])
        coords[1] = float(src[1])
    
    if isinstance(dest, Vec2d):
        return Vec2d(coords[0], coords[1])
    elif isinstance(dest, np.ndarray):
        return np.array([np.float64(coords[0]), np.float64(coords[1])])

class Physics:
    # world: RectangularWorld
    def __init__(self, world):
        self.world = world
        self.space = pymunk.Space()
        self.mass = 1
        self.radius = 0.1

    def step(self):
        self.space.step(self.world.dt)
        for agent in self.world.population:
            if not hasattr(agent, "physobj"):
                continue
            newPos = copyCoords(agent.pos, agent.physobj.position)
            agent.dpos = newPos - agent.pos
            agent.dtheta = agent.physobj.angular_velocity
            agent.pos = newPos
            agent.angle = np.float64(agent.physobj.angle)

    def createAgentShape(self, agent):
        friction = 0.5
        if hasattr(agent, "friction"):
            friction = agent.friction
        mass = 1
        if hasattr(agent, "mass"):
            mass = agent.mass
        shape: pymunk.Shape = pymunk.shapes.Circle(body=None, radius=0.001)
        if hasattr(agent, "points") and 0 < agent.points.size:
            shape = pymunk.shapes.Poly(body=None, vertices=agent.points.tolist())
        elif hasattr(agent, "radius"):
            shape = pymunk.shapes.Circle(body=None, radius=agent.radius)
        
        shape.mass = mass
        shape.friction = friction
        return shape

    def createAgentBody(self, agent):
        body = pymunk.Body()
        shape = self.createAgentShape(agent)
        shape.body = body
        body.position = copyCoords(body.position, agent.pos)
        body.angle = float(agent.angle)
        self.space.add(body, shape)
        return body
    
    def createStaticBody(self, agent):
        shape = self.createAgentShape(agent)
        shape.body = self.space.static_body
        shape.elasticity = 0.8
        self.space.add(shape)
        # return body

# def kineticFriction(body: pymunk.Body, coof, dt): # body and coefficient of friction
#     g = 9.8
#     peakdv = g * coof * dt
#     effectiveFriction = body.mass * g * coof
#     if body.velocity.length < peakdv:
#         effectiveFriction *= body.velocity.length / peakdv
    
#     unit = body.velocity.normalized()
#     if unit.length == 0:
#         return Vec2d.zero()
#     return unit.scale_to_length(-effectiveFriction)
def lsq(vec: Vec2d):
    return vec.x**2 + vec.y**2

def getOrtho(vec: Vec2d, omega):
    # <-yo, xo, 0>
    if lsq(vec) == 0:
        return Vec2d.zero()
    return Vec2d(-vec.y * omega, vec.x * omega).normalized()

peakAgentForceCache = {}

def peakAgentForce(mass, velocity, omega):
    #caching here!
    argTup = (mass, velocity, omega)
    if argTup in peakAgentForceCache:
        return peakAgentForceCache[argTup]

    fakeBody = pymunk.Body(mass=mass)
    fakeBody.velocity = Vec2d(velocity, 0)
    # kf = kineticFriction(fakeBody, 1, 0)
    fcmag = fakeBody.mass * velocity * omega
    if fcmag == 0:
        return 0
    fc = getOrtho(fakeBody.velocity, omega).scale_to_length(fcmag)
    
    peakAgentForceCache[argTup] = (fc).length # caching
    return (fc).length

def agentForces(body: pymunk.Body, velocity, omega, peakForce, dt):
    # kf = friction
    intendedVector = Vec2d(velocity, 0)
    vDiff = intendedVector - body.world_to_local(body.velocity + body.position)
    vDirF = vDiff * (1/dt) * body.mass
    if peakForce < vDirF.length: # prioritize getting up to speed in the pointed direction
        return vDirF.scale_to_length(peakForce)
    # vDirF -= kf
    # if peakForce < vDirF.length: # next prioritize counteracting friction
        # return vDirF.scale_to_length(peakForce)
    ortho = getOrtho(intendedVector, omega)
    turnF = Vec2d.zero() if lsq(ortho) == 0 else ortho.scale_to_length(body.mass * velocity * omega)
    vDirF += turnF
    if peakForce < vDirF.length: # next prioritize turning force
        return vDirF.scale_to_length(peakForce)
    return vDirF

def peakAgentTorque(body: pymunk.Body, omega, dt):
    return abs(omega) * 2 / dt * body.moment

def agentTorques(body: pymunk.Body, velocity, omega, peakTorque, dt):
    avDiff = omega - body.angular_velocity
    avDirT = avDiff / dt * body.moment
    return float(np.clip(avDirT, -peakTorque, peakTorque))

def snapPhysicsToAgent(agent):
    agent.physobj.position = copyCoords(agent.physobj.position, agent.pos)
    agent.physobj.angle = float(agent.angle)
    agent.physobj.velocity = copyCoords(agent.physobj.velocity, agent.getVelocity())

def unicycleForces(body: pymunk.Body, v, omega, dt, peakV, peakOmega):
    peakVelocity = peakV #0.3
    peakOmega = peakOmega #2

    # friction = kineticFriction(body, 1, self.dt)
    # body.apply_force_at_local_point(friction)
    peakForce = peakAgentForce(body.mass, peakVelocity, peakOmega)
    force = agentForces(body, v, omega, peakForce, dt)
    body.apply_force_at_local_point(force)

    torque = agentTorques(body, v, omega, peakAgentTorque(body, peakOmega, dt), dt)
    body.apply_force_at_local_point((0, torque), (1, 0))
    body.apply_force_at_local_point((0, -torque))