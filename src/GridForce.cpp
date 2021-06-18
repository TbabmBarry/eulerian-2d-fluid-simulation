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

// void GridForce::apply(bool springsCanBreak)
// {
//     if (this->active)
//     {
//         for (Particle* p : particles) {
//             float sx = 180 / 900;
// 			float sy = 100 / 500;

// 			Particle* rigidBody = p;
// 			float pixels = rigidBody.getGridCells();

// 			for(int i = 0; i < pixels.length; i++) {
// 				float rowSize = (sx * rigidBody.width * 2);
// 				float x = i % rowSize;
// 				float y = floor(i / rowSize);

// 				if (pixels[i] != 0) {
// 					// 0 When cell before is part of object, 1 when it is surrounding
// 					float before = (((i - 1) % rowSize) >= 1)? 1 - min(1,pixels[i - 1]) : 0;
// 					float after =  (((i % rowSize) + 1) <= (rowSize - 1))? 1 - min(1,pixels[i + 1]) : 0;
// 					float above =  ((i - rowSize) >= 0)? 1 - min(1,pixels[i - rowSize]) : 0;
// 					float below =  ((i + rowSize) <= (pixels.length-1))? 1 - min(1,pixels[i + rowSize]) : 0;
					
// 					float rx = round(sx * (rigidBody.position.x - rigidBody.width) + x);
// 					float ry = round(sy * (rigidBody.position.y - rigidBody.height) + y);
// 					float j = ry * rowSize + rx; //index of cell in total grid

// 					float u = scene.fluidField.u;
// 					float v = scene.fluidField.v;
// 					float d = scene.fluidField.dens;

// 					//Sample velocities around point
// 					float localVelocities = [];
// 					if(before == 1) localVelocities.push([scene.fluidField.getXVelocity(rx-1,ry), scene.fluidField.getYVelocity(rx-1,ry), scene.fluidField.getDensity(rx-1,ry)]);
// 					if(after == 1) localVelocities.push([scene.fluidField.getXVelocity(rx+1,ry), scene.fluidField.getYVelocity(rx+1,ry), scene.fluidField.getDensity(rx+1,ry)]);
// 					if(above == 1) localVelocities.push([scene.fluidField.getXVelocity(rx,ry-1), scene.fluidField.getYVelocity(rx,ry-1), scene.fluidField.getDensity(rx,ry-1)]);
// 					if(below == 1) localVelocities.push([scene.fluidField.getXVelocity(rx,ry+1), scene.fluidField.getYVelocity(rx,ry+1), scene.fluidField.getDensity(rx,ry+1)]);

// 					localVelocities = localVelocities.filter(function(v) {
// 						return !isNaN(v[0]) && !isNaN(v[1]) && isFinite(v[0]) && isFinite(v[1]);
// 					});

// 					if(localVelocities.length == 0)
// 						continue;


// 					//For every pixel on the edge of the object
// 					if(before+after+above+below > 0){

// 						//Compute the normal of the surface
// 						float rrx = (rigidBody.position.x - rigidBody.width) + ((x+0.5)/sx);
// 						float rry = (rigidBody.position.y - rigidBody.height) + ((y+0.5)/sy);

// 						//Find closest edge
// 						float closestEdge = rigidBody.getClosestEdge(rrx, rry)[0];
// 						float closestEdgeVector = [
// 							closestEdge[0][0] - closestEdge[1][0],
// 							closestEdge[0][1] - closestEdge[1][1]
// 						];

// 						//Find normal of closest edge
// 						float normal = [closestEdgeVector[1], -closestEdgeVector[0]];
// 						float length = sqrt(normal[0]*normal[0] + normal[1]*normal[1]);
// 						normal[0] = (normal[0] == 0) ? 0 : normal[0] / length;
// 						normal[1] = (normal[1] == 0) ? 0 : normal[1] / length;

						
// 						//Find distance to center
// 						float surface = [-normal[1], normal[0]]; //perpendicular to normal
// 						float helper = [
// 							((rx/sx) - rigidBody.position.x),
// 							((ry/sy) - rigidBody.position.y)
// 						];

// 						//project along the surface
// 						helper[0] *= surface[0];
// 						helper[1] *= surface[1];

// 						float distanceToCenter = sqrt(helper[0]*helper[0] + helper[1]*helper[1]);
// 						distanceToCenter *= (helper[0]+helper[1] < 0) ? distanceToCenter : -distanceToCenter; //difference clockwise / anticlockwise
// 						distanceToCenter /= 100;

// 						//project velocities along the normal
// 						float totalVelocity = localVelocities.map(function(v) {
// 							return [v[0]*normal[0], v[1]*normal[1], v[2]];
// 						});
// 						float totalVelocity = localVelocities.reduce(function(v1, v2) {
// 							return [v1[0] + v2[0], v1[1] + v2[1], v1[2] + v2[2]];
// 						});

// 						float velocityLength = min(1, sqrt(totalVelocity[0]*totalVelocity[0] + totalVelocity[1]*totalVelocity[1]));
// 						float localDensity = totalVelocity[2];

// 						//Display force lines
// 						float arrowlength = (abs(distanceToCenter) * velocityLength)*2; //velocityLength*20 + 1;
// 						float point = [rrx+(normal[0]*arrowlength), rry+(normal[1]*arrowlength)];
// 						graphics.lineStyle(3, 0xff0000, 1);
// 						graphics.moveTo(rrx, rry);
// 						graphics.lineTo(point[0], point[1]);

// 						// float pressure = (0.5*localDensity) * (velocityLength*velocityLength);
// 						// rigidBody.torque += (distanceToCenter*pressure)/70;
// 						// rigidBody.force.x -= pressure*5;
// 						// rigidBody.force.y -= pressure*5;
// 						rigidBody.torque += (distanceToCenter * velocityLength)/100;
// 						rigidBody.force.x += (totalVelocity[0] * velocityLength)*7;
// 						rigidBody.force.y += (totalVelocity[1] * velocityLength)*7;

// 					}
// 				}
// 			}
//         }
//     }
    
// }

void GridForce::draw()
{

}