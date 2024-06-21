#include "simulate.h"
#include "planar_quadrotor_visualizer.h"
#include <matplot/matplot.h>

SDL_AudioDeviceID gAudioDeviceID;
bool gAudioInitialized = false;
std::vector<Uint8> gAudioBuffer;
Uint32 gAudioBufferPosition = 0;

void audioCallback(void* userdata, Uint8* stream, int len) {
    if (gAudioBuffer.empty()) {
        return;
    }

    Uint32 remainingBytes = gAudioBuffer.size() - gAudioBufferPosition;
    Uint32 bytesToCopy = std::min(static_cast<Uint32>(len), remainingBytes);

    std::memcpy(stream, &gAudioBuffer[gAudioBufferPosition], bytesToCopy);

    gAudioBufferPosition += bytesToCopy;

    if (gAudioBufferPosition >= gAudioBuffer.size()) {
        gAudioBufferPosition = 0;
    }
}

bool loadAudioFile(const std::string& filename) {
    
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Failed to open audio file: " << filename << std::endl;
        return false;
    }

    // Read all data from the file into the buffer
    file.seekg(0, std::ios::end);
    std::streampos fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    gAudioBuffer.resize(fileSize);
    file.read(reinterpret_cast<char*>(&gAudioBuffer[0]), fileSize);
    file.close();

    return true;
}

int initAudio(const std::string& audioFilename) {
    if (SDL_Init(SDL_INIT_AUDIO) < 0) {
        std::cerr << "SDL audio initialization failed: " << SDL_GetError() << std::endl;
        return -1;
    }

    if (!loadAudioFile(audioFilename)) {
        std::cerr << "Failed to load audio file: " << audioFilename << std::endl;
        return -1;
    }

    SDL_AudioSpec desiredSpec, obtainedSpec;
    SDL_zero(desiredSpec);
    desiredSpec.freq = 44100;
    desiredSpec.format = AUDIO_S16SYS;
    desiredSpec.channels = 1; 
    desiredSpec.samples = 2048;
    desiredSpec.callback = audioCallback;

    gAudioDeviceID = SDL_OpenAudioDevice(nullptr, 0, &desiredSpec, &obtainedSpec, 0);
    if (gAudioDeviceID == 0) {
        std::cerr << "Failed to open audio device: " << SDL_GetError() << std::endl;
        return -1;
    }

    SDL_PauseAudioDevice(gAudioDeviceID, 0); // Start playing audio

    gAudioInitialized = true;
    return 0;
}

Eigen::MatrixXf LQR(PlanarQuadrotor& quadrotor, float dt) {
    Eigen::MatrixXf Eye = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf A_discrete = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf B(6, 2);
    Eigen::MatrixXf B_discrete(6, 2);
    Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf R = Eigen::MatrixXf::Identity(2, 2);
    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(6, 6);
    Eigen::Vector2f input = quadrotor.GravityCompInput();

    Q.diagonal() << 0.004, 0.004, 400 * M_PI, 0.005, 0.045, 2 / 2 / M_PI;
    R.row(0) << 30, 7;
    R.row(1) << 7, 30;

    std::tie(A, B) = quadrotor.Linearize();
    A_discrete = Eye + dt * A;
    B_discrete = dt * B;

    return LQR(A_discrete, B_discrete, Q, R);
}

void control(PlanarQuadrotor& quadrotor, const Eigen::MatrixXf& K) {
    Eigen::Vector2f input = quadrotor.GravityCompInput();
    quadrotor.SetInput(input - K * quadrotor.GetControlState());
}

int main(int argc, char* args[]) {
    std::shared_ptr<SDL_Window> gWindow = nullptr;
    std::shared_ptr<SDL_Renderer> gRenderer = nullptr;
    const int SCREEN_WIDTH = 1280;
    const int SCREEN_HEIGHT = 720;

    Eigen::VectorXf initial_state = Eigen::VectorXf::Zero(6);
    PlanarQuadrotor quadrotor(initial_state);
    PlanarQuadrotorVisualizer quadrotor_visualizer(&quadrotor);

    Eigen::VectorXf goal_state = Eigen::VectorXf::Zero(6);
    goal_state << 0, 0, 0, 0, 0, 0;
    quadrotor.SetGoal(goal_state);
    const float dt = 0.01;
    Eigen::MatrixXf K = LQR(quadrotor, dt);
    Eigen::Vector2f input = Eigen::Vector2f::Zero(2);

    if (init(gWindow, gRenderer, SCREEN_WIDTH, SCREEN_HEIGHT) >= 0) {

        SDL_Event event;
        bool quit = false;
        float delay;
        int x, y;
        Eigen::VectorXf state = Eigen::VectorXf::Zero(6);

        int target_x = 0;
        int target_y = 0;
        bool target_active = false;
        float previous_theta = 0;
        float current_theta = 0;

        std::string audioFilename = "drone_soundeffect2.wav";
        if (initAudio(audioFilename) < 0) {
            std::cerr << "Failed to initialize SDL audio." << std::endl;
            return -1;
        }

        while (!quit) {
            while (SDL_PollEvent(&event) != 0) {
                if (event.type == SDL_QUIT) {
                    quit = true;
                }
                else if (event.type == SDL_MOUSEBUTTONDOWN && event.button.button == SDL_BUTTON_LEFT) {
                    SDL_GetMouseState(&target_x, &target_y);
                    target_active = true;

                    std::cout << '\n' << "| Current position is: " << '\n';
                    std::cout << "| X: " << target_x << '\n';
                    std::cout << "| Y: " << target_y << '\n';

                    float new_goal_x = (target_x - SCREEN_WIDTH / 2);
                    float new_goal_y = -(target_y - SCREEN_HEIGHT / 2);

                    std::cout << '|' << '\n' << "| Going to: " << '\n';
                    std::cout << "| X: " << new_goal_x << '\n';
                    std::cout << "| Y: " << new_goal_y << '\n';

                    goal_state << new_goal_x, new_goal_y, 0, 0, 0, 0;
                    quadrotor.SetGoal(goal_state);
                }
                else if (event.type == SDL_KEYDOWN) {
                    if (event.key.keysym.sym == SDLK_p) {
                        using namespace matplot;

                        figure();

                        subplot(3, 1, 1);
                        plot(quadrotor_visualizer.x_history);
                        title("X History:");

                        subplot(3, 1, 2);
                        plot(quadrotor_visualizer.y_history);
                        title("Y History:");

                        subplot(3, 1, 3);
                        plot(quadrotor_visualizer.theta_history);
                        title("Theta History:");

                        show();
                    }
                }
            }

            SDL_Delay((int)(dt * 100));

            SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
            SDL_RenderClear(gRenderer.get());

            Eigen::VectorXf state = quadrotor.GetState();
            float screen_x = SCREEN_WIDTH / 2 + state[0];
            float screen_y = SCREEN_HEIGHT / 2 - state[1];
            float distance_to_target = sqrt(pow(screen_x - target_x, 2) + pow(screen_y - target_y, 2));

            if (target_active) {
                SDL_SetRenderDrawColor(gRenderer.get(), 0x00, 0x00, 0x00, 0xFF);
                SDL_RenderDrawLine(gRenderer.get(), target_x - 5, target_y - 5, target_x + 5, target_y + 5);
                SDL_RenderDrawLine(gRenderer.get(), target_x - 5, target_y + 5, target_x + 5, target_y - 5);
            }

            if (distance_to_target < 60) {
                target_active = false;
            }

            quadrotor_visualizer.render(gRenderer);

            SDL_RenderPresent(gRenderer.get());

            control(quadrotor, K);
            quadrotor.Update(dt);
        }

        if (gAudioInitialized) {
            SDL_CloseAudioDevice(gAudioDeviceID);
            SDL_QuitSubSystem(SDL_INIT_AUDIO);
            gAudioInitialized = false;
        }
    }
    SDL_Quit();
    return 0;
}

int init(std::shared_ptr<SDL_Window>& gWindow, std::shared_ptr<SDL_Renderer>& gRenderer, const int SCREEN_WIDTH, const int SCREEN_HEIGHT)
{
    if (SDL_Init(SDL_INIT_VIDEO) >= 0) {
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
        gWindow = std::shared_ptr<SDL_Window>(SDL_CreateWindow("Planar Quadrotor", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN), SDL_DestroyWindow);
        gRenderer = std::shared_ptr<SDL_Renderer>(SDL_CreateRenderer(gWindow.get(), -1, SDL_RENDERER_ACCELERATED), SDL_DestroyRenderer);
        SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
    }
    else {
        std::cout << "SDL_ERROR: " << SDL_GetError() << std::endl;
        return -1;
    }
    return 0;
}
