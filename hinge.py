
# Originally by Matthias Baas.
# Updated by Pierre Gay to work without pygame or cgkit.

import sys, os, random, time
from math import *
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

import ode

# geometric utility functions
def scalp (vec, scal):
    vec[0] *= scal
    vec[1] *= scal
    vec[2] *= scal

def length (vec):
    return sqrt (vec[0]**2 + vec[1]**2 + vec[2]**2)

# prepare_GL
def prepare_GL():
    """Prepare drawing.
    """

    # Viewport
    glViewport(0,0,640,480)

    # Initialize
    glClearColor(0.8,0.8,0.9,0)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glEnable(GL_CULL_FACE)
    glEnable(GL_DEPTH_TEST)
    glDisable(GL_LIGHTING)
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
    glLightfv(GL_LIGHT0,GL_DIFFUSE,[1,1,1,1])
    glLightfv(GL_LIGHT0,GL_SPECULAR,[1,1,1,1])
    glEnable(GL_LIGHT0)

    # View transformation
    gluLookAt (2.4, 3.6, 4.8, 0.5, 0.5, 0, 0, 1, 0)

# draw_body
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

def drop_object():
    """Drop an object into the scene."""

    global joints, bodies, geom

    body, geom = create_box(world, space, 1000, 1, 1, 1)
    body.setPosition((0, 0.5, 0))
    bodies.append(body)
    geoms.append(geom)

    body2, geom2 = create_box(world, space, 100, 0.5, 0.5, 0.75)
    body2.setPosition((0.75, 0.75, 0))
    bodies.append(body2)
    geoms.append(geom2)

    body3, geom3 = create_box(world, space, 100, 0.4, 0.4, 0.5)
    body3.setPosition((1.2, 0.8, 0))
    bodies.append(body3)
    geoms.append(geom3)

    hinge2 = ode.HingeJoint(world)
    hinge2.attach(body, body2)
    hinge2.setAnchor((0.5, 1, 0))
    hinge2.setAxis((0, 0, 1))
    hinge2.setParam(ode.ParamFMax, 10000)

    hinge = ode.HingeJoint(world)
    hinge.attach(body2, body3)
    hinge.setAnchor((1.0, 1, 0))
    hinge.setAxis((0, 0, 1))
    hinge.setParam(ode.ParamFMax, 10000)

    joints.append(hinge2)
    joints.append(hinge)

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
glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE)

x = 0
y = 0
width = 640
height = 480
glutInitWindowPosition (x, y);
glutInitWindowSize (width, height);
glutCreateWindow ("testode")

# Create a world object
world = ode.World()
world.setGravity( (0,-9.81,0) )
#world.setERP(0.8)
#world.setCFM(1E-5)

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

drop_object()

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

glutKeyboardFunc (_keyfunc)

# draw callback
def _drawfunc ():
    # Draw the scene
    prepare_GL()
    for b in bodies:
        draw_body(b)

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

