#include "ros/ros.h"
#include <string.h>
#include <algorithm>

#include "keyboard_control/KeyboardStates.h"

#include <SDL/SDL.h>

ros::Publisher g_KeyboardStatesPub;

keyboard_control::KeyboardStates g_KeyboardStatesMsg;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "keyboard_control");
	ros::NodeHandle n;
	g_KeyboardStatesPub = n.advertise<keyboard_control::KeyboardStates>("/keyboard_control/KeyboardStates", 1);
	
	SDL_Surface *screen;
	SDL_Event event;

	if (SDL_Init(SDL_INIT_VIDEO) < 0)
		return 1;

	if (!(screen = SDL_SetVideoMode(100, 100, 32, SDL_HWSURFACE))) {
		SDL_Quit();
		printf("SDL_SetVideoMode failed to open window\n");
		return 1;
	}

	SDL_WM_SetCaption("KeyboardSates", "KeyboardStates");

    bool quit = false;
	ros::Rate loop_rate(30); // frequency
	while (ros::ok() && !quit) {
		while (SDL_PollEvent(&event)) {
			switch (event.type) {
			case SDL_QUIT:
				quit = true;
				break;
			case SDL_KEYDOWN:
			case SDL_KEYUP:
			{
                if (event.key.keysym.sym == SDLK_LEFT) g_KeyboardStatesMsg.bLeftArrow = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_RIGHT) g_KeyboardStatesMsg.bRightArrow = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_UP) g_KeyboardStatesMsg.bUpArrow = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_DOWN) g_KeyboardStatesMsg.bDownArrow = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_0) g_KeyboardStatesMsg.b0 = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_1) g_KeyboardStatesMsg.b1 = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_2) g_KeyboardStatesMsg.b2 = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_3) g_KeyboardStatesMsg.b3 = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_4) g_KeyboardStatesMsg.b4 = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_5) g_KeyboardStatesMsg.b5 = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_6) g_KeyboardStatesMsg.b6 = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_7) g_KeyboardStatesMsg.b7 = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_8) g_KeyboardStatesMsg.b8 = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_9) g_KeyboardStatesMsg.b9 = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_a) g_KeyboardStatesMsg.bA = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_b) g_KeyboardStatesMsg.bB = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_c) g_KeyboardStatesMsg.bC = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_d) g_KeyboardStatesMsg.bD = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_e) g_KeyboardStatesMsg.bE = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_f) g_KeyboardStatesMsg.bF = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_g) g_KeyboardStatesMsg.bG = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_h) g_KeyboardStatesMsg.bH = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_i) g_KeyboardStatesMsg.bI = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_j) g_KeyboardStatesMsg.bJ = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_k) g_KeyboardStatesMsg.bK = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_l) g_KeyboardStatesMsg.bL = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_m) g_KeyboardStatesMsg.bM = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_n) g_KeyboardStatesMsg.bN = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_o) g_KeyboardStatesMsg.bO = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_p) g_KeyboardStatesMsg.bP = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_q) g_KeyboardStatesMsg.bQ = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_r) g_KeyboardStatesMsg.bR = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_s) g_KeyboardStatesMsg.bS = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_t) g_KeyboardStatesMsg.bT = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_u) g_KeyboardStatesMsg.bU = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_v) g_KeyboardStatesMsg.bV = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_w) g_KeyboardStatesMsg.bW = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_x) g_KeyboardStatesMsg.bX = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_y) g_KeyboardStatesMsg.bY = (event.type == SDL_KEYDOWN);
                if (event.key.keysym.sym == SDLK_z) g_KeyboardStatesMsg.bZ = (event.type == SDL_KEYDOWN);
				g_KeyboardStatesPub.publish(g_KeyboardStatesMsg);
				break;
			}
			}
		}

		loop_rate.sleep(); // Sleep for 10 ms
		ros::spinOnce(); // Process ros messages
	}

	SDL_Quit();
	return EXIT_SUCCESS;
}

