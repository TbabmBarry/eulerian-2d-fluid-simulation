#include "GridForce.h"
#include <math.h>

GridForce::GridForce(vector<Particle *> particles, Fluid *fluid, int scale)
    : fluid(fluid), scale(scale)
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
      // cout<<rb->torque<<endl;
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
          edges.push_back(rb->corners[(i + 1) % (rb->corners.size())] -
                          rb->corners[i % (rb->corners.size())]);
          normals.push_back(Vector2f(
              edges[i][1], -edges[i][0])); // point to inner area of rigid body
        }
        Vector2f on_edge;
        Vector2f normal;
        for (int i = 0; i < normals.size(); ++i)
        {
          if (CenterToGrid.dot(normals[i]) / (CenterToGrid.norm() * normals[i].norm()) <= -sqrt(2) / 2)
          {
            on_edge = edges[i];
            normal =
                normals[i].normalized(); // point to inner area of rigid body
          }
        }
        // project velocities along the normal and sum up
        // because we want to simply assume force as a vector proportional to velocity
        float projectedVelocity =
            (u * on_edge[1] - on_edge[0] * v) / on_edge.norm();
        Vector2f localVelocity = projectedVelocity * normal;

        rb->m_Force -= localVelocity;                           // force = alpha*v
        rb->torque -= CenterToGrid.dot(Vector2f(u, v)) * scale; // torque = L*v
      }
      // rb->m_Force = rb->m_Force.normalized() * 10000;
      // cout<<rb->m_Force<<endl;
    }
  }
}

map<int, map<int, float>> GridForce::dx()
{
  return map<int, map<int, float>>();
}

MatrixXf GridForce::dv() { return MatrixXf(); }

void GridForce::draw() {}