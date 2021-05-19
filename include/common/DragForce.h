#include "Force.h"

class DragForce : public Force {
    public:
        DragForce(vector<Particle*> particles, float drag_k);

        void setTarget(vector<Particle*> particles) override;
        void apply(bool springsCanBreak) override;
        void draw() override;
    private:
        float drag_k;
};