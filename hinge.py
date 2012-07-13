
# Based on PyODE tutorials by Matthias Baas and Pierre Gay.

import time
from math import *
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

import ode

from geometry import V3

def prepare_GL():
    # Viewport
    glViewport(0,0,640,480)

    # Initialize
    glEnable(GL_CULL_FACE)
    glEnable(GL_DEPTH_TEST)
    glClearColor(0.8,0.8,0.9,0)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    glEnable(GL_LIGHTING)
    glEnable(GL_NORMALIZE)
    glShadeModel(GL_FLAT)

    # Projection
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective (45,1.3333,0.2,20)

    # Initialize ModelView matrix
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

    # Light source
    glLightfv(GL_LIGHT0,GL_POSITION,[0,0,1,0])
    glLightfv(GL_LIGHT0,GL_DIFFUSE,[0.8, 0.7, 0.5, 1])
    glLightfv(GL_LIGHT0,GL_SPECULAR,[1,1,1,1])
    glEnable(GL_LIGHT0)

    glLightfv(GL_LIGHT1,GL_POSITION,[0.5,1,0.2,0])
    glLightfv(GL_LIGHT1,GL_DIFFUSE,[0.5, 0.7, 0.8, 1])
    glLightfv(GL_LIGHT1,GL_SPECULAR,[1,1,1,1])
    glEnable(GL_LIGHT1)

    # View transformation
    gluLookAt (2.4, 3.6, 4.8, 0.5, 0.5, 0, 0, 1, 0)

def draw_body(body):
    """Draw an ODE body.
    """
    x,y,z = body.getPosition()
    R = body.getRotation()
    rot = [R[0], R[3], R[6], 0.,
           R[1], R[4], R[7], 0.,
           R[2], R[5], R[8], 0.,
           x, y, z, 1.0]
    glPushMatrix()
    glMultMatrixd(rot)
    if body.shape=="box":
        sx,sy,sz = body.boxsize
        glScalef(sx, sy, sz)
        glutSolidCube(1)
    glPopMatrix()

def draw_joint(joint):
    x,y,z = joint.getAnchor()
    rot = [1, 0, 0, 0.,
           0, 1, 0, 0.,
           0, 0, 1, 0.,
           x, y, z, 1.0]
    glPushMatrix()
    glMultMatrixd(rot)
    glScalef(0.1, 0.1, 0.1)
    glutSolidCube(1)
    glPopMatrix()

def create_box(world, space, density, lx, ly, lz):
    """Create a box body and its corresponding geom."""

    # Create body
    body = ode.Body(world)
    M = ode.Mass()
    M.setBox(density, lx, ly, lz)
    body.setMass(M)

    # Set parameters for drawing the body
    body.shape = "box"
    body.boxsize = (lx, ly, lz)

    # Create a box geom for collision detection
    geom = ode.GeomBox(space, lengths=body.boxsize)
    geom.setBody(body)

    return body, geom

def create_agent():
    """Create and drop an agent into the world"""
    global joints, bodies, geom

    body_dimensions = V3(0.1, 0.5, 0.5)
    body_position = V3(0, 3, 0)
    gap = V3(0.05, 0, 0)
    anchor_position = V3(body_dimensions.x/2.0 + gap.x/2.0, 3, 0)

    for body_number in range(30):
        body, geom = create_box(world, space, 1000, *body_dimensions)
        body.setPosition(body_position)

        if body_number > 0:
            last_body = bodies[-1]
            hinge = ode.HingeJoint(world)
            hinge.attach(last_body, body)
            hinge.setAnchor(anchor_position)
            hinge.setAxis((0, 0, 1))
            hinge.setParam(ode.ParamFMax, 10000)
            joints.append(hinge)
            anchor_position += V3(body_dimensions.x + gap.x, 0, 0)

        body_position += V3(body_dimensions.x + gap.x, 0, 0)
        bodies.append(body)
        geoms.append(geom)

# Collision callback
def near_callback(args, geom1, geom2):
    """Callback function for the collide() method.

    This function checks if the given geoms do collide and
    creates contact joints if they do.
    """

    # Check if the objects do collide
    contacts = ode.collide(geom1, geom2)

    # Create contact joints
    world,contactgroup = args
    for c in contacts:
        c.setBounce(0.04)
        c.setMu(500)
        j = ode.ContactJoint(world, contactgroup, c)
        j.attach(geom1.getBody(), geom2.getBody())


glutInit ([])
glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH)

x = 0
y = 0
width = 640
height = 480
glutInitWindowPosition(x, y)
glutInitWindowSize(width, height)
glutCreateWindow("testode")

# Create a world object
world = ode.World()
world.setGravity( (0,-9.81,0) )

# Create a space object
space = ode.Space()

# Create a plane geom which prevent the objects from falling forever
floor = ode.GeomPlane(space, (0,1,0), 0)

# A list with ODE bodies
bodies = []
joints = []

# The geoms for each of the bodies
geoms = []

# A joint group for the contact joints that are generated whenever
# two bodies collide
contactgroup = ode.JointGroup()

# Some variables used inside the simulation loop
fps = 50
dt = 1.0/fps
running = True
state = 0
counter = 0
lasttime = time.time()

create_agent()

# keyboard callback
def _keyfunc (c, x, y):
    if c == 'z':
        joints[0].setParam(ode.ParamVel, -3.0)
    if c == 'x':
        joints[0].setParam(ode.ParamVel, 3.0)
    if c == ' ':
        joints[0].setParam(ode.ParamVel, 0.0)
        joints[1].setParam(ode.ParamVel, 0.0)

    if c == 'a':
        joints[1].setParam(ode.ParamVel, -3.0)
    if c == 's':
        joints[1].setParam(ode.ParamVel, 3.0)

def _update(t):
    for (i, joint) in enumerate(joints):
        joints[i].setParam(ode.ParamVel, 4 * sin(t + (i * 0.7)))
    glutTimerFunc (50, _update, t+1)

glutKeyboardFunc (_keyfunc)
glutTimerFunc (50, _update, 1)

# draw callback
def _drawfunc ():
    # Draw the scene
    prepare_GL()
    for b in bodies:
        draw_body(b)
    for j in joints:
        draw_joint(j)

    glutSwapBuffers ()

glutDisplayFunc (_drawfunc)

# idle callback
def _idlefunc ():
    global counter, state, lasttime

    t = dt - (time.time() - lasttime)
    if (t > 0):
        time.sleep(t)

    glutPostRedisplay ()

    space.collide((world,contactgroup), near_callback)
    world.step(dt)
    contactgroup.empty()

    lasttime = time.time()

glutIdleFunc (_idlefunc)
glutMainLoop ()
