#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include "shaders.h"
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "Physics.h"
#include "glSetup.h"
#include <algorithm>
#include "imgui.h"
#include "implot.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#define grav 9.81f

	int main() {
		Simulation Sim{};
		Sim.setTimeStep(0.01f);

		// Per-link parameter arrays — all must be size n
		unsigned int n{ 30 };
		float* length{ new float[n] {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1} };

		// Sum of link lengths = max reach of the chain. We size the sim space to
		// this so a fully-extended pendulum still fits in the window
		float totalLength{};
		for (unsigned int i{}; i < n; i++)
			totalLength += length[i];

		// Window is in pixels; sim space is in physics units. The vertex shader
		// scales sim coords by 1/simX, 1/simY into NDC
		GLuint windowX{ 800 }, windowY{ 800 };
		float simX{ totalLength };

		float simY{ simX * (static_cast<float>(windowY) / static_cast<float>(windowX)) };

		// Anchor near the top so the chain has room to swing below it.
		glm::vec3 pivot{ 0.0f, simY * 0.8f, 0.0f };

		float* initAng{ new float[n] {1.0f, 0.5f, 0.3f, 1.0f, 0.5f, 0.3f, 1.0f, 0.5f, 0.3f, 0.0f, 1.0f, 0.5f, 0.3f, 1.0f, 0.5f, 0.3f, 1.0f, 0.5f, 0.3f, 0.0f, 1.0f, 0.5f, 0.3f, 1.0f, 0.5f, 0.3f, 1.0f, 0.5f, 0.3f, 0.0f} };
		float* initAngVel{ new float[n] {0.1f, 0.0f, 5.0f, 1.0f, 0.5f, 0.3f, 1.0f, 0.5f, 0.3f,0.0f, 0.1f, 0.0f, 5.0f, 1.0f, 0.5f, 0.3f, 1.0f, 0.5f, 0.3f,0.0f, 0.1f, 0.0f, 5.0f, 1.0f, 0.5f, 0.3f, 1.0f, 0.5f, 0.3f,0.0f} };
		unsigned int* nodes{ new unsigned int[n] {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10} };  // triangle-fan resolution per bob
		float* radius{ new float[n] {0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f} };

		DataStore data{ Sim.setUpPend(n, pivot, length, initAng, initAngVel, radius, nodes, windowX, windowY, simX, simY) };

		GLuint shaderProgram{ initShaders() };

		// Simulation has copied these into its own state, so the setup arrays are done
		delete[] length;
		delete[] initAng;
		delete[] initAngVel;
		delete[] nodes;
		delete[] radius;

		GLuint* VBO{ data.getVBO() };
		GLuint* VAO{ data.getVAO() };
		GLFWwindow* window{ data.getWindow() };
		GLfloat prevTime{};

		// Two model matrices per link: Ctran positions the bob, Ltran positions
		// and rotates the connecting line about its pivot
		glm::mat4* Ctran{ new glm::mat4[n] };
		glm::mat4* Ltran{ new glm::mat4[n] };

		for (unsigned int i = 0; i < n; i++) {
			Ctran[i] = glm::translate(glm::mat4(1.0f), Sim.getPendArray()[i].getPos());
			Ltran[i] = glm::translate(glm::mat4(1.0f), Sim.getPendArray()[i].getPivot());
			Ltran[i] = glm::rotate(Ltran[i], Sim.getPendArray()[i].getAngle(), glm::vec3(0.0f, 0.0f, 1.0f));
		}

		int translation;

		// ImGui + ImPlot for the live energy / Lagrangian plots
		IMGUI_CHECKVERSION();
		ImGui::CreateContext();
		ImPlot::CreateContext();
		ImGui::StyleColorsDark();
		ImGui_ImplGlfw_InitForOpenGL(window, true);
		ImGui_ImplOpenGL3_Init("#version 330");

		// Rolling 100-sample buffers — index 99 is always the newest sample,
		// older samples slide left each physics step
		float* time_data{ new float[100] {} };
		float* UE_data{ new float[100] {} };  // potential V
		float* KE_data{ new float[100] {} };  // kinetic   T
		float* L_data{ new float[100] {} };   // Lagrangian T - V
		float* E_data{ new float[100] {} };   // total      T + V  (should be ~constant if integrator is good)

		while (!glfwWindowShouldClose(window)) {
			ImGui_ImplOpenGL3_NewFrame();
			ImGui_ImplGlfw_NewFrame();
			ImGui::NewFrame();

			glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
			glUseProgram(shaderProgram);
			glClear(GL_COLOR_BUFFER_BIT);

			translation = glGetUniformLocation(shaderProgram, "translate");

			// Sim-to-NDC scale doesn't change between draws, so set the uniform once
			int scaleLoc = glGetUniformLocation(shaderProgram, "scale");
			glm::mat4 scale{ glm::scale(glm::mat4(1.0f), glm::vec3(1.0f / simX, 1.0f / simY, 1.0f)) };
			glUniformMatrix4fv(scaleLoc, 1, GL_FALSE, glm::value_ptr(scale));

			// Draw bobs first, then the lines on top — keeps line ends from looking
			// like they're floating outside the circles
			for (unsigned int i{}; i < n; i++) {
				glUniformMatrix4fv(translation, 1, GL_FALSE, glm::value_ptr(Ctran[i]));
				glBindVertexArray(VAO[i * 2]);
				glDrawArrays(GL_TRIANGLE_FAN, 0, Sim.getPendArray()[i].getCircleVertices());
			}
			for (unsigned int i{}; i < n; i++) {
				glUniformMatrix4fv(translation, 1, GL_FALSE, glm::value_ptr(Ltran[i]));
				glBindVertexArray(VAO[i * 2 + 1]);
				glDrawArrays(GL_LINES, 0, 2);
			}

			// advance the simulation every 10 ms
			if (glfwGetTime() - prevTime > 0.01f) {
				Sim.RK4Step();

				// Refresh the per-link transforms now that positions/angles changed
				for (unsigned int i{}; i < n; i++) {
					Ctran[i] = glm::translate(glm::mat4(1.0f), Sim.getPendArray()[i].getPos());
					Ltran[i] = glm::translate(glm::mat4(1.0f), Sim.getPendArray()[i].getPivot());
					Ltran[i] = glm::rotate(Ltran[i], Sim.getPendArray()[i].getAngle(), glm::vec3(0.0f, 0.0f, 1.0f));
				}

				prevTime = static_cast<GLfloat>(glfwGetTime());

				// Slide the ring buffer left so index 99 is free for the new sample
				for (unsigned int i{ 1 }; i < 100; i++) {
					UE_data[i - 1] = UE_data[i];
					KE_data[i - 1] = KE_data[i];
					L_data[i - 1] = L_data[i];
					E_data[i - 1] = E_data[i];
					time_data[i - 1] = time_data[i];
				}

				KE_data[99] = Sim.calcKinetic();
				UE_data[99] = Sim.calcPotential();
				E_data[99] = KE_data[99] + UE_data[99];
				L_data[99] = KE_data[99] - UE_data[99];
				time_data[99] = static_cast<float>(glfwGetTime());
			}

			ImGui::Begin("Graph");

			// Scroll the X axis with the newest sample so the plot stays "live"
			float currentTime = time_data[99];
			float historyWindow = 1.5f;  // seconds shown at once

			if (ImPlot::BeginPlot("Energy")) {
				ImPlot::SetupAxisLimits(ImAxis_X1, currentTime - historyWindow, currentTime, ImGuiCond_Always);
				ImPlot::PlotLine("Potential Energy", time_data, UE_data, 100);
				ImPlot::PlotLine("Kinetic Energy", time_data, KE_data, 100);
				ImPlot::PlotLine("Total Energy", time_data, E_data, 100);
				ImPlot::EndPlot();
			}
			if (ImPlot::BeginPlot("Lagrangian")) {
				ImPlot::SetupAxisLimits(ImAxis_X1, currentTime - historyWindow, currentTime, ImGuiCond_Always);
				ImPlot::PlotLine("Lagrangian", time_data, L_data, 100);
				ImPlot::EndPlot();
			}
			ImGui::End();

			ImGui::Render();
			ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

			glfwSwapBuffers(window);
			glfwPollEvents();
			glBindVertexArray(0);
		}

		ImGui_ImplOpenGL3_Shutdown();
		ImGui_ImplGlfw_Shutdown();
		ImPlot::DestroyContext();
		ImGui::DestroyContext();

		data.glClean();
		delete[] Ctran;
		delete[] Ltran;
		return 0;
	}