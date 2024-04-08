#pragma once

#include <givio.h>
#include <givr.h>
#include <imgui/imgui.h>

namespace imgui_panel {
	extern bool showPanel;
	extern ImVec4 clear_color;
	extern bool reset_view;

	//Simulation settings
	extern int number_of_iterations_per_frame;
	extern bool play_simulation;
	extern bool reset_simulation;
	extern bool step_simulation;
	extern float dt_simulation;
	extern float ks;
	extern float ka;
	extern float kc;
	extern float rs;
	extern float ra;
	extern float rc;
	extern float ds;
	extern float da;
	extern float dc;


	// lambda function
	extern std::function<void(void)> draw;
} // namespace panel