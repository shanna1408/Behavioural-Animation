#include "imgui_panel.hpp"

namespace imgui_panel {
	// default values
	bool showPanel = true;
	ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
	bool reset_view = false;
	

	//Simulation settings
	int number_of_iterations_per_frame = 1;
	bool play_simulation = false;
	bool reset_simulation = false;
	bool step_simulation = false;
	float dt_simulation = 0.001f;
	float ks = 1.f;
	float ka = 0.4f;
	float kc = 0.2f;
	float rs = 6.f;
	float ra = 8.f;
	float rc = 5.f;
	float ds = 175.f;
	float da = 140.f;
	float dc = 120.f;


	std::function<void(void)> draw = [](void) {
		if (showPanel && ImGui::Begin("Panel", &showPanel, ImGuiWindowFlags_MenuBar)) {
			ImGui::Spacing();
			ImGui::Separator();

			ImGui::ColorEdit3("Clear color", (float*)&clear_color);
			reset_view = ImGui::Button("Reset View");

			ImGui::Spacing();
			ImGui::Separator();

			ImGui::SliderInt("Iterations Per Frame", &number_of_iterations_per_frame, 1, 100);
			ImGui::Checkbox("Play Simulation", &play_simulation);
			reset_simulation = ImGui::Button("Reset Simulation");
			if (!play_simulation) {
				step_simulation = ImGui::Button("Step Simulation");
			}
			ImGui::DragFloat("Simulation dt", &dt_simulation, 1.e-5f, 1.e-5f, 1.f, "%.6e");

			ImGui::Separator();

			float frame_rate = ImGui::GetIO().Framerate;
			ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
				1000.0f / frame_rate, frame_rate);

			ImGui::Spacing();
			ImGui::Separator();

			ImGui::DragFloat("ks (Separation)", &ks, 0.001f, 0.f, 5.f, "%.3f");
			ImGui::DragFloat("ka (Alignment)", &ka, 0.001f, 0.f, 5.f, "%.3f");
			ImGui::DragFloat("kc (Cohesion)", &kc, 0.001f, 0.f, 5.f, "%.3f");

			ImGui::Spacing();
			ImGui::Separator();

			ImGui::DragFloat("rs (Separation)", &rs, 0.01f, 0.f, 10.f, "%.2f");
			ImGui::DragFloat("ra (Alignment)", &ra, 0.01f, 0.f, 10.f, "%.2f");
			ImGui::DragFloat("rc (Cohesion)", &rc, 1.f, 0.f, 10.f, "%1.f");

			ImGui::Spacing();
			ImGui::Separator();

			ImGui::DragFloat("Separation Angle", &ds, 1.f, 0.f, 180.f, "%1.f deg");
			ImGui::DragFloat("Alignment Angle", &da, 1.f, 0.f, 180.f, "%1.f deg");
			ImGui::DragFloat("Cohesion Angle", &dc, 1.f, 0.f, 180.f, "%1.f deg");

			ImGui::Spacing();
			ImGui::Separator();

			ImGui::End();
		}
	};
} // namespace panel