#pragma once

#include <memory>
#include <SDL.h>
#include "planar_quadrotor.h"

class PlanarQuadrotorVisualizer {
private:
    PlanarQuadrotor *quadrotor_ptr;
    float previous_theta;
public:

    std::vector<float> x_history;
    std::vector<float> y_history;
    std::vector<float> theta_history;

    PlanarQuadrotorVisualizer(PlanarQuadrotor *quadrotor_ptr);
    void render(std::shared_ptr<SDL_Renderer> &gRenderer);
};