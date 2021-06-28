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

void GridForce::applyFluidF(FluidSolver* fluid)
{
    if (this->active)
    {
        for (Particle* p : particles) {

			Particle* rigidBody = p;
			vector<Vector4f> grids = rigidBody->BoundingGrid(128);

            // vector<Vector4f> temp = rigidbodies[i]->BoundingGrid(8);
            // cout << "new " << endl;
            // for (int i=0;i<temp.size();i++) {
            // cout << "bound_grid " << temp[i][0] << " " <<temp[i][1]<< " "<<temp[i][2]<< " " <<temp[i][3]<< endl;
            // }
            cout<<"ciao"<<endl;
			for(int i = 0; i < grids.size(); i++) {
                cout<<"ciao1"<<endl;
                // use v,F of the grids that edges pass through to update
                cout<<typeid(fluid->rigidbodies[0]).name()<<endl;
                cout<<(fluid->aaa)<<endl;
                float u = fluid->getXVelocity((int)grids[i][0],(int)grids[i][1]);
                float v = fluid->getYVelocity((int)grids[i][0],(int)grids[i][1]);
                float density = fluid->getDensity((int)grids[i][0],(int)grids[i][1]);
                Vector2f CenterToGrid = Vector2f(grids[i][2],grids[i][3]);

                //Find which edge is current grid allocated on
                vector<Vector2f> edges;
                vector<Vector2f> normals;
                for(int i=0;i<rigidBody->corners.size();++i){
                    edges[i]=rigidBody->corners[(i+1)%(rigidBody->corners.size())]-rigidBody->corners[i%(rigidBody->corners.size())];
                    normals[i]=Vector2f(edges[i][1],-edges[i][0]);//point to inner area of rigid body
                }
                Vector2f on_edge;
                Vector2f normal;
                for(int i=0;i<normals.size();++i){
                    if(CenterToGrid.dot(normals[i])>0){
                        on_edge=edges[i];
                        normal=normals[i].normalized();//point to inner area of rigid body
                    }
                }
                //project velocities along the normal and sum up
                //because we want to simply assume force as a vector proportional to velocity
                float projectedVelocity = u * on_edge[1] - on_edge[0] * v/on_edge.norm();
                Vector2f localVelocity = projectedVelocity * normal;
                
                rigidBody->force += 7 * localVelocity;//force = alpha*v
                rigidBody->torque += CenterToGrid.dot(7 * localVelocity);//torque = L*v
			}
        }
    }
    
}

void GridForce::apply(bool springsCanBreak)
{
}

map<int, map<int, float>> GridForce::dx()
{
    return map<int, map<int, float>>();
}

MatrixXf GridForce::dv()
{
    return MatrixXf();
}

void GridForce::draw()
{

}