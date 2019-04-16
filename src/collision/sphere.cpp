#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/sphere_drawing.h"
#include "sphere.h"

using namespace nanogui;
using namespace CGL;

void Sphere::collide(PointMass &pm) {
  // TODO (Part 3): Handle collisions with spheres.
	if ((pm.position - origin).norm() < radius) { //pm is "inside sphere", needs correction.
		//Extend path between position, origin to sphere surface
		Vector3D extension = (pm.position - origin).unit()*radius;
		//Correction pt from last_pos to extension:
		Vector3D correction = extension - pm.last_position;
		correction += origin; //MAKE SURE STARTS @ ORIGIN
		pm.position = (friction * pm.last_position) + (pm.last_position + correction)*(1 - friction);
	}
}

void Sphere::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  m_sphere_mesh.draw_sphere(shader, origin, radius * 0.92);
}
