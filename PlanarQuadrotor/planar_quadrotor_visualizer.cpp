#include "planar_quadrotor_visualizer.h"

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor* quadrotor_ptr)
    : quadrotor_ptr(quadrotor_ptr) {}

void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer>& gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    float q_x = state[0];
    float q_y = state[1];
    float q_theta = state[2];

    int screen_width, screen_height;
    SDL_GetRendererOutputSize(gRenderer.get(), &screen_width, &screen_height);
    float center_x = screen_width / 2;
    float center_y = screen_height / 2;

    float visual_x = center_x + q_x;
    float visual_y = center_y - q_y;

    SDL_Texture* body_texture = SDL_CreateTexture(gRenderer.get(), SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STATIC, 1, 1);
    constexpr Uint32 pixel = 0xFFFFFFFF; // White color
    SDL_UpdateTexture(body_texture, nullptr, &pixel, sizeof(pixel));
    SDL_SetTextureColorMod(body_texture, 0, 0, 0);

    int bodyRect_x = visual_x - 50;
    int bodyRect_y = visual_y - 10;
    int bodyRect_width = 100;
    int bodyRect_height = 20;

    const SDL_Rect body_rect = { static_cast<int>(bodyRect_x), static_cast<int>(bodyRect_y), bodyRect_width, bodyRect_height };
    SDL_RenderCopyEx(gRenderer.get(), body_texture, nullptr, &body_rect, -q_theta * (180 / M_PI), nullptr, SDL_FLIP_NONE);
}
