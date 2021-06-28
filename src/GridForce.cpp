#include "GridForce.h"
#include <math.h>

GridForce::GridForce(vector<Particle *> particles, FluidSolver *fluid)
    : fluid(fluid)
{
  this->setTarget(particles);
}

void GridForce::setTarget(vector<Particle *> particles)
{
  this->particles = particles;
}

void GridForce::apply(bool springsCanBreak)
{
  if (this->active)
  {
    for (Particle *rb : particles)
    {
      vector<Vector4f> grids = rb->BoundingGrid(128);

      // vector<Vector4f> temp = rigidbodies[i]->BoundingGrid(8);
      // cout << "new " << endl;
      // for (int i=0;i<temp.size();i++) {
      // cout << "bound_grid " << temp[i][0] << " " <<temp[i][1]<< "
      // "<<temp[i][2]<< " " <<temp[i][3]<< endl;
      // }

      for (int i = 0; i < grids.size(); i++)
      {
        // use v,F of the grids that edges pass through to update
        float u = fluid->getXVelocity((int)grids[i][0], (int)grids[i][1]);
        float v = fluid->getYVelocity((int)grids[i][0], (int)grids[i][1]);
        float density = fluid->getDensity((int)grids[i][0], (int)grids[i][1]);
        Vector2f CenterToGrid = Vector2f(grids[i][2], grids[i][3]);

        // Find which edge is current grid allocated on
        vector<Vector2f> edges;
        vector<Vector2f> normals;
        for (int i = 0; i < rb->corners.size(); ++i)
        {
          edges[i] = rb->corners[(i + 1) % (rb->corners.size())] -
                     rb->corners[i % (rb->corners.size())];
          normals[i] = Vector2f(
              edges[i][1], -edges[i][0]); // point to inner area of rigid body
        }
        Vector2f on_edge;
        Vector2f normal;
        for (int i = 0; i < normals.size(); ++i)
        {
          if (CenterToGrid.dot(normals[i]) > 0)
          {
            on_edge = edges[i];
            normal =
                normals[i].normalized(); // point to inner area of rigid body
          }
        }
        // project velocities along the normal and sum up
        // because we want to simply assume force as a vector proportional to
        // velocity
        float projectedVelocity =
            (u * on_edge[1] - on_edge[0] * v) / on_edge.norm();
        Vector2f localVelocity = projectedVelocity * normal;

        rb->force += 7 * localVelocity;                    // force = alpha*v
        rb->torque += CenterToGrid.dot(7 * localVelocity); // torque = L*v
      }
    }
  }
}

map<int, map<int, float>> GridForce::dx()
{
  return map<int, map<int, float>>();
}

MatrixXf GridForce::dv() { return MatrixXf(); }

void GridForce::draw() {}