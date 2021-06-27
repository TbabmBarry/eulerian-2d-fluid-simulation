#include "GridForce.h"
#include <math.h>

GridForce::GridForce(vector<Particle*> particles)
{
    this->setTarget(particles);
}

void GridForce::setTarget(vector<Particle*> particles)
{
    this->particles = particles;
}

void GridForce::apply(FluidSolver fluid)
{
    if (this->active)
    {
        Vector2f totalVelocity=Vector2f(0,0);
        for (Particle* p : particles) {

			Particle* rigidBody = p;
			vector<Vector4f> grids;// = rigidBody->BoundingGrid(128);

			for(int i = 0; i < grids.size(); i++) {
                // use v,F of the grids that edges pass through to update
                float u = fluid.getXVelocity((int)grids[i][0],(int)grids[i][1]);
                float v = fluid.getYVelocity((int)grids[i][0],(int)grids[i][1]);
                float density = fluid.getDensity((int)grids[i][0],(int)grids[i][1]);
                Vector2f distanceToCenter = Vector2f(grids[i][2],grids[i][3]);

                //Find closest edges
                vector<Vector2f> close_edge = rigidBody->getClosestEdge(Vector2f(grids[1][2],grids[i][3]));//bottomright->topright
                Vector2f edge=close_edge[1]-close_edge[0];
                //Find normal of closest edge
                Vector2f normal = Vector2f(edge[1]/rigidBody->dimension, -edge[0]/rigidBody->dimension);

                //project velocities along the normal and sum up
                //because we want to simply assume force as a vector proportional to velocity
                Vector2f localVelocity = Vector2f(u * normal[0], v * normal[1]);
                totalVelocity += localVelocity;
                float velocityabs = sqrt(totalVelocity[0]*totalVelocity[0] + totalVelocity[1]*totalVelocity[1]);
                float localDensity = totalVelocity[2];

                rigidBody->torque += distanceToCenter.dot(localVelocity);//torque = L*v
                rigidBody->force += (totalVelocity * velocityabs)*7;//force = alpha*v
			}
        }
    }
    
}

void GridForce::draw()
{

}