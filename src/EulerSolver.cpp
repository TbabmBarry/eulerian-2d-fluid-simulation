#include "EulerSolver.h"
#include "Eigen/Dense"
#include <Eigen/Sparse>
#include "Eigen/IterativeLinearSolvers"

EulerSolver::EulerSolver(EulerSolver::TYPE type) : type(type) {}

void EulerSolver::simulateStep(System *system, float h) {

    switch (type)
    {
    case EXPLICIT:
        explicitS(system, h);
        break;
    case IMPLICIT:
        implicitS(system, h);
        break;
    case SEMI:
        semiS(system, h);
    }

}

void EulerSolver::explicitS(System *system, float h) {

    // Get the old state
    VectorXf oldState = system->particleGetState();

    // Evaluate derivative
    VectorXf stateDeriv = system->particleAcceleration();
    // Calculate the new state
    VectorXf newState = oldState + h * stateDeriv;

    if (system->wall)
        newState = system->collisionValidation(newState);
    //set the new state
    system->particleSetState(newState, system->particleGetTime() + h);
}

void EulerSolver::semiS(System *system, float h) {
    // Get the old state
    VectorXf oldState = system->particleGetState();

    // Evaluate derivative
    VectorXf stateDeriv = system->particleAcceleration();

    // Calculate the new state
    VectorXf newState = oldState + h * stateDeriv;
    //set the new state
    system->particleSetState(newState, system->particleGetTime() + h);

    //Use new vectory
    // VectorXf semiState(system->particleDims());
    for (int i = 0; i < newState.size(); i += 4) {
        newState[i + 0] = oldState[i + 0] + h * newState[i + 2];
        newState[i + 1] = oldState[i + 1] + h * newState[i + 3];
    }

    if (system->wall)
        newState = system->collisionValidation(newState);
    //set the new state
    system->particleSetState(newState, system->particleGetTime() + h);            
}


void EulerSolver::implicitS(System *system, float h) {
    // Get the old state
    VectorXf oldState = system->particleGetState();
    system->particleAcceleration();
    // Fill mass matrix
    SparseMatrix<float> M(system->particleDims() / 2, system->particleDims() / 2);

    vector<Triplet<float>> MtripletList;
    MtripletList.reserve(system->particleDims() / 2);
    for (int i = 0; i < system->particles.size() * 2; i += 2) {
        MtripletList.push_back(Triplet<float>(i+0, i+0, system->particles[i / 2]->mass));
        MtripletList.push_back(Triplet<float>(i+1, i+1, system->particles[i / 2]->mass));
    }
    M.setFromTriplets(MtripletList.begin(), MtripletList.end());

    // Fill jx and jy matrix based
    SparseMatrix<float> jx(system->particleDims() / 2, system->particleDims() / 2);
    MatrixXf jv(system->particleDims() / 2, system->particleDims() / 2);

    // Initialize empty map to compute jx
    auto jxm = map<int, map<int, float>>();
    unsigned long entries = 0;
    for (Force *f : system->forces) {
        // Compute map for every force and update jxm appropriately
        auto fjx = f->dx();
        for (const auto &i1 : fjx) {
            for (const auto &i2 : i1.second) {
                if (jxm.count(i1.first) && jxm[i1.first].count(i2.first)) {
                    // todo maybe check if we can remove this, it should never happen?
                    // i1 and i2 exist
                    // Hence, we update the already existing value
                    jxm[i1.first][i2.first] += i2.second;
                } else {
                    // No value yet exists, since i1 or i2 does not exist
                    // Hence we set a new value
                    jxm[i1.first][i2.first] = i2.second;
                    entries++;
                }
            }
        }

        if (f->particles.size() == 2) {
            MatrixXf fjv = f->dv();
            jv.block(f->particles[0]->index * 2, f->particles[1]->index * 2, fjv.cols(), fjv.rows()) = fjv;
            jv.block(f->particles[1]->index * 2, f->particles[0]->index * 2, fjv.cols(), fjv.rows()) = fjv;
        }
    }

    vector<Triplet<float>> JxTripletList;
    JxTripletList.reserve(entries);
    for (const auto &i1 : jxm) {
        for (const auto &i2 : i1.second) {
            JxTripletList.push_back(Triplet<float>(i1.first, i2.first, i2.second));
        }
    }
    jx.setFromTriplets(JxTripletList.begin(), JxTripletList.end());

    // Get fold and vold
    SparseVector<float> fold(system->particleDims() / 2);
    SparseVector<float> vold(system->particleDims() / 2);


    for (int i = 0; i < system->particles.size(); i++) {
        Particle *p = system->particles[i];
        vold.coeffRef(i * 2 + 0) = p->m_Velocity[0];
        vold.coeffRef(i * 2 + 1) = p->m_Velocity[1];
        fold.coeffRef(i * 2 + 0) = p->m_Force[0];
        fold.coeffRef(i * 2 + 1) = p->m_Force[1];
    }

    // Compute A
    SparseMatrix<float> A = M - h * h * jx; // - h * jv;
    SparseVector<float> b = h * (fold + h * jx * vold);
    // Solve for dy
    ConjugateGradient<SparseMatrix<float>, Lower|Upper> cg;
    cg.compute(A);
    SparseVector<float> dy = cg.solve(b);

//    std::cout << "dv:     " << dy << std::endl;

    // Set new state
    VectorXf newState(system->particleDims());
    for (int i = 0; i < dy.size(); i += 2) {
        int si = i * 2; // State index
        newState[si + 0] = oldState[si + 0] + (oldState[si + 2] + dy.coeff(i+0)) * h;    // dX = (V0 + dV) * h
        newState[si + 1] = oldState[si + 1] + (oldState[si + 3] + dy.coeff(i+1)) * h;
        newState[si + 2] = oldState[si + 2] + dy.coeff(i+0);// * h;        // Update velocity
        newState[si + 3] = oldState[si + 3] + dy.coeff(i+1);// * h;
    }

    //set the new state
    system->particleSetState(newState, system->particleGetTime() + h);
}
    
