#include "ConstraintMaintainer.h"
#include "Eigen/Dense"
#include "Eigen/IterativeLinearSolvers"


void ConstraintMaintainer::maintainConstraint(System *system, float m_ks, float m_kd)
{

    std::cout << "in maintainConstraint "<< std::endl;
    vector<Particle*> particles = system->particles;
    vector<Constraint*> constraints = system->constraints;

    // Check if no constraint is applied
    if (constraints.empty()) return;

    // Dimension of a particle in the system
    int particleDim = 2;
    size_t particleSysSize = particles.size() * particleDim;

    // Initiate mass matrix and inverse mass matrix for the system
    MatrixXf M = MatrixXf::Zero(particleSysSize, particleSysSize);
    MatrixXf W = MatrixXf::Zero(particleSysSize, particleSysSize);

    // Initiate known forces and first time derivative of particles' configuration for the system
    VectorXf Q = VectorXf::Zero(particleSysSize);
    VectorXf qDot = VectorXf::Zero(particleSysSize);

    for (int i = 0; i < particleSysSize; i += particleDim)
    {
        Particle *p = particles[i / particleDim];
        for (int j = 0; j < particleDim; j++)
        {
            Q[i+j] = p->m_Force[j];
            qDot[i+j] = p->m_Velocity[j];
            M(i+j, i+j) = p->mass;
            W(i+j, i+j) = 1 / p->mass;
        }
    }

    size_t constraintSize = constraints.size();

    // Initiate constraint and first time derivative constraint vector
    VectorXf C = VectorXf::Zero(constraintSize);
    VectorXf CDot = VectorXf::Zero(constraintSize);

    // Initiate jacobian, first time derivative jacobian, and the transpose of jacobian matrix
    MatrixXf J = MatrixXf::Zero(constraintSize, particleSysSize);
    MatrixXf JDot = MatrixXf::Zero(constraintSize, particleSysSize);
    MatrixXf Jt = MatrixXf::Zero(particleSysSize, constraintSize);

    for (int i = 0; i < constraintSize; i++)
    {
        // Obtain a specific constraint from the vector
        Constraint *c = constraints[i];

        // Retrieve and store the constraint
        C[i] = c->C();
        // Retrieve and store the the legal velocity of a particular particle 
        CDot[i] = c->legalVelocity();
        // Retrieve and store the jacobian vector
        vector<Vec2f> jacobian = c->jacobian();
        // Retrieve the first time derivative of the jacobian vector
        vector<Vec2f> jacobianDerivative = c->jacobianDerivative();

        vector<Particle*> constraintsParticleVector = c->store();

        for (int j = 0; j < constraintsParticleVector.size(); j++)
        {
            int idx = constraintsParticleVector[j]->index * particleDim;
            for (int k = 0; k < particleDim; k++)
            {
                JDot(i, idx+k) = jacobianDerivative[j][k];
                J(i, idx+k) = jacobian[j][k];
                Jt(idx+k, i) = jacobian[j][k];
            }
        }
    }


    MatrixXf JW = J * W, JWJt = JW * Jt;
    VectorXf ksC = m_ks * C, kdCDot = m_kd * CDot, JDotqDot = JDot * qDot, JWQ = JW * Q;

    // Gather and compute the right hand side object to do conjugate gradient
    VectorXf b = JDotqDot - JWQ - ksC - kdCDot;
    // std::cout<<J<<std::endl;
    // std::cout<<' '<<std::endl;
    // std::cout<<W<<std::endl;
    // std::cout<<' '<<std::endl;
    // std::cout<<Jt<<std::endl;
    // std::cout<<' '<<std::endl;
    // std::cout<<JWJt<<std::endl;
    // std::cout<<' '<<std::endl;
    // std::cout<<b<<std::endl;
    // std::cout<<' '<<std::endl;
    ConjugateGradient<MatrixXf, Lower|Upper> cg;
    cg.compute(JWJt);
    VectorXf lambda = cg.solve(b);
    // Compute the constraint force Q hat
    VectorXf QHat = Jt * lambda;
    for (int i = 0; i < particles.size(); i++)
    {
        Particle *p = particles[i];
        int idx = particleDim * i;
        for (int j = 0; j < particleDim; j++)
        {
            p->m_Force[j] += QHat[idx + j];
        }

        std::cout << "during force" << particles[i]->m_Force << std::endl;
        std::cout << "during velocity" << particles[i]->m_Velocity << std::endl;
    }
}
