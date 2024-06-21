#include "planar_quadrotor_visualizer.h"
#include <cmath>   // For math functions and constants

#include "SDL2_gfx/SDL2_gfxPrimitives.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2 (M_PI / 2.0)
#endif

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor* quadrotor_ptr)
    : quadrotor_ptr(quadrotor_ptr), previous_theta(0) {}

float interpolate_theta(float previous_theta, float current_theta) {

    float angle_difference = current_theta - previous_theta;
    
    if (angle_difference > M_PI) {
        angle_difference -= 2 * M_PI;
        return previous_theta + angle_difference * 0.005f;
    }

    if (angle_difference < -M_PI) {
        angle_difference += 2 * M_PI;
        return previous_theta + angle_difference * 0.005f;
    }

    return previous_theta + angle_difference * 0.005f;
}

void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer>& gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    float q_x, q_y, q_theta;
    q_x = state[0];
    q_y = state[1];
    q_theta = state[2];

    q_theta = interpolate_theta(previous_theta, q_theta);
    previous_theta = q_theta;

    int screen_width, screen_height;
    SDL_GetRendererOutputSize(gRenderer.get(), &screen_width, &screen_height);

    int elipse_width = 80;
    int elipse_height = 20;
    int wing_height = 20;
    int propeller_radius = 10;

    float center_x = screen_width / 2;
    float center_y = screen_height / 2;

    float visual_x = center_x + q_x;
    float visual_y = center_y - q_y;

    int screen_x = center_x + static_cast<int>(q_x);
    int screen_y = center_y - static_cast<int>(q_y);

    SDL_Texture* texture = SDL_CreateTexture(gRenderer.get(), SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET, elipse_width + 2 * wing_height, elipse_height + wing_height);
    SDL_SetRenderTarget(gRenderer.get(), texture);
    SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
    SDL_RenderClear(gRenderer.get());


    filledEllipseRGBA(gRenderer.get(), wing_height + elipse_width / 2, wing_height / 2 + elipse_height / 2, elipse_width / 2, elipse_height / 2, 0xFF, 0xA5, 0x00, 0xFF);
    ellipseRGBA(gRenderer.get(), wing_height + elipse_width / 2, wing_height / 2 + elipse_height / 2, elipse_width / 2, elipse_height / 2, 0xFF, 0xA5, 0x00, 0xFF);

    SDL_SetRenderDrawColor(gRenderer.get(), 0x70, 0x80, 0x90, 0xFF);
    SDL_Rect left_wing = { 0, 0, wing_height, wing_height };
    SDL_Rect right_wing = { elipse_width + wing_height, 0, wing_height, wing_height };

    SDL_RenderFillRect(gRenderer.get(), &left_wing);
    SDL_RenderFillRect(gRenderer.get(), &right_wing);

    SDL_SetRenderDrawColor(gRenderer.get(), 0x00, 0x00, 0x00, 0xFF); // Black color for propellers

    int left_propeller_center_x = left_wing.x + wing_height / 2;
    int left_propeller_center_y = left_wing.y + wing_height / 2;

    int right_propeller_center_x = right_wing.x + wing_height / 2;
    int right_propeller_center_y = right_wing.y + wing_height / 2;

    float angle = SDL_GetTicks() / 100.0f; // Change the speed of rotation by modifying the divisor

    for (int i = 0; i < 4; ++i) {
        float propeller_angle = angle + i * M_PI_2;

        int left_propeller_x = left_propeller_center_x + static_cast<int>(propeller_radius * cos(propeller_angle));
        int left_propeller_y = left_propeller_center_y + static_cast<int>(propeller_radius * sin(propeller_angle));

        int right_propeller_x = right_propeller_center_x + static_cast<int>(propeller_radius * cos(propeller_angle));
        int right_propeller_y = right_propeller_center_y + static_cast<int>(propeller_radius * sin(propeller_angle));

        SDL_RenderDrawLine(gRenderer.get(), left_propeller_center_x, left_propeller_center_y, left_propeller_x, left_propeller_y);
        SDL_RenderDrawLine(gRenderer.get(), right_propeller_center_x, right_propeller_center_y, right_propeller_x, right_propeller_y);
    }

    SDL_SetRenderTarget(gRenderer.get(), nullptr);

    SDL_Rect goal_rectangle = { screen_x - (elipse_width + 2 * wing_height) / 2, screen_y - (elipse_height + wing_height) / 2, elipse_width + 2 * wing_height, elipse_height + wing_height };
    SDL_Point center = { (elipse_width + 2 * wing_height) / 2, (elipse_height + wing_height) / 2 };

    SDL_RenderCopyEx(gRenderer.get(), texture, nullptr, &goal_rectangle, -q_theta * 180.0 / M_PI, &center, SDL_FLIP_NONE);

    SDL_DestroyTexture(texture);

    SDL_RenderPresent(gRenderer.get());
}
