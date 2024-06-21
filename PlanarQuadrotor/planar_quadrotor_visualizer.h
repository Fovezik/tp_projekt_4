#pragma once

#include <memory>
#include <SDL.h>
#include "planar_quadrotor.h"

class PlanarQuadrotorVisualizer {
private:
    PlanarQuadrotor *quadrotor_ptr;
    float previous_theta;
public:
    PlanarQuadrotorVisualizer(PlanarQuadrotor *quadrotor_ptr);
    void render(std::shared_ptr<SDL_Renderer> &gRenderer);
};