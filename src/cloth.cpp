#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.
	//Initialize vectors
	point_masses = vector<PointMass>{};
	springs = vector<Spring>{};
	//Populate point_masses
	double width_scale = width / num_width_points;
	double height_scale = height / num_height_points;
	for (int y = 0; y < num_height_points; y++) {
		for (int x = 0; x < num_width_points; x++) {
			int index = y * num_width_points + x;
			double offset = (((double)rand() / RAND_MAX) * 0.002) - 0.001;
			Vector3D position = (orientation == HORIZONTAL) ? Vector3D(x*width_scale, 1, y*height_scale) : Vector3D(x*width_scale, y*height_scale, offset);
			bool pin = false;
			PointMass curr = PointMass(position, pin);
			point_masses.push_back(curr);
		}
	}
	//Set pinned to true
	for (int i = 0; i < pinned.size(); i++) {
		int index = pinned[i][1] * num_width_points + pinned[i][0];
		point_masses[index].pinned = true;
	}
	//Populate springs
	for (int y = 0; y < num_height_points; y++) {
		for (int x = 0; x < num_width_points; x++) {
			//indices:
			int curr = y * num_width_points + x;
			int prev = y * num_width_points + (x - 1); //structural
			int up = (y - 1) * num_width_points + x; //structural
			int up_left = (y - 1) * num_width_points + (x - 1); //shear
			int up_right = (y - 1) * num_width_points + (x + 1); //shear
			int skip_left = y * num_width_points + (x - 2); //bend
			int skip_up = (y - 2) * num_width_points + x; //bend
			//springs:
			if ((x - 1) >= 0) { //prev
				springs.push_back(Spring(&point_masses[curr], &point_masses[prev], STRUCTURAL));
			}
			if ((y - 1) >= 0) {
				springs.push_back(Spring(&point_masses[curr], &point_masses[up], STRUCTURAL));
				if ((x - 1) >= 0) {
					springs.push_back(Spring(&point_masses[curr], &point_masses[up_left], SHEARING));
				}
				if ((x + 1) < num_width_points) {
					springs.push_back(Spring(&point_masses[curr], &point_masses[up_right], SHEARING));
				}
			}
			if ((x - 2) >= 0) {
				springs.push_back(Spring(&point_masses[curr], &point_masses[skip_left], BENDING));
			}
			if ((y - 2) >= 0) {
				springs.push_back(Spring(&point_masses[curr], &point_masses[skip_up], BENDING));
			}
		}
	}
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // TODO (Part 2): Compute total force acting on each point mass.
  
	//Total external force
  Vector3D total_ext_f = 0;
  for (int i = 0; i < external_accelerations.size(); i++) {
	  total_ext_f += external_accelerations[i];
  }
	//Apply force to every point mass, F = m*a
  for (int i = 0; i < point_masses.size(); i++) {
	  point_masses[i].forces = total_ext_f * mass;
  }
	//Apply spring correction forces
  for (int i = 0; i < springs.size(); i++) {
	  double force = 0;
	  if (cp->enable_structural_constraints || cp->enable_shearing_constraints) {
		  force = cp->ks * ((springs[i].pm_a->position - springs[i].pm_b->position).norm() - springs[i].rest_length);
	  }
	  if (cp->enable_bending_constraints) {
		  force = 0.2 * cp->ks * ((springs[i].pm_a->position - springs[i].pm_b->position).norm() - springs[i].rest_length);
	  }
	  Vector3D force_vect = (springs[i].pm_b->position - springs[i].pm_a->position).unit() * force;
	  springs[i].pm_a->forces += force_vect;
	  springs[i].pm_b->forces -= force_vect;
  }

  // TODO (Part 2): Use Verlet integration to compute new point mass positions
  for (int i = 0; i < point_masses.size(); i++) {
	  if (!(point_masses[i].pinned)) { //if not pinned, update position
		  Vector3D prevPos = point_masses[i].last_position;
		  Vector3D currPos = point_masses[i].position;
		  double damp_factor = 1.0 - (cp->damping / 100.0);
		  Vector3D accel = point_masses[i].forces / mass;
		  Vector3D newPos = currPos + damp_factor * (currPos - prevPos) + accel * pow(delta_t, 2);
		  point_masses[i].last_position = currPos;
		  point_masses[i].position = newPos;
	  }
  }/**/

  // TODO (Part 4): Handle self-collisions.
  build_spatial_map();
  for (int i = 0; i < point_masses.size(); i++) {
	  self_collide(point_masses[i], simulation_steps);
  }

  // TODO (Part 3): Handle collisions with other primitives.
  for (int j = 0; j < (*collision_objects).size(); j++) {
	  for (int i = 0; i < point_masses.size(); i++) {
		  (*collision_objects)[j]->collide(point_masses[i]);
	  }
  }

  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
  for (int i = 0; i < springs.size(); i++) {
	  Vector3D a_pos = springs[i].pm_a->position; //pointmass a pos
	  Vector3D b_pos = springs[i].pm_b->position; //pointmass b pos
	  double curr_length = (b_pos - a_pos).norm();
	  double max_length = springs[i].rest_length * 1.1;

	  double diff = curr_length - max_length;
	  Vector3D dir = (b_pos - a_pos);

	  if (curr_length > max_length) { //Not within 10%
		  if (springs[i].pm_a->pinned) { //Pinned
			  springs[i].pm_b->position -= dir.unit() * diff;
		  }
		  else if (springs[i].pm_b->pinned) { //Pinned
			  springs[i].pm_a->position += dir.unit() * diff;
		  }
		  else { //Not pinned
			  springs[i].pm_a->position += dir.unit() * 0.5 * diff;
			  springs[i].pm_b->position -= dir.unit() * 0.5 * diff;
		  }
	  }
  }/**/
}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.
  for (int i = 0; i < point_masses.size(); i++) { //over all point masses
	  float key = hash_position(point_masses[i].position);
	  if (map.end() == map.find(key)) { //key not in map
		  map[key] = new vector<PointMass*>();
	  }
	  map[key]->push_back(&point_masses[i]);
  }
}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.
	Vector3D correction = Vector3D(0, 0, 0);
	int count = 0;
	float key = hash_position(pm.position);
	vector<PointMass*> values = *(map[key]);
	for (int i = 0; i < values.size(); i++) {
		PointMass* candidate = values[i];
		if (&pm != candidate) {
			//double distance = (pm.position - candidate->position).norm();
			double distance = (candidate->position - pm.position).norm();
			if (distance <= (2 * thickness)) { //within self collision range
				//correction += (candidate->position - pm.position).unit() * (distance - (2 * thickness));
				correction += (pm.position - candidate->position).unit() * ((2 * thickness) - distance);
				count++;
			}
		}
	}
	if (count) {
		correction /= count;
	}
	if (simulation_steps) {
		correction /= simulation_steps;
	}
	pm.position += correction;
}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
	float w = 3 * width / num_width_points;
	float h = 3 * height / num_height_points;
	float t = max(w, h);
	
	int x_num = floor(pos.x/ w);
	int y_num = floor(pos.y/ h);
	int z_num = floor(pos.z/ t);/**/

	float box = x_num * pow(113, 2) + y_num * 113 + z_num;
	//cout << "box: " << box << "\n";
	return box;
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
