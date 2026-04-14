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

int main() {
    Simulation Sim{};
	// Set simulation timestep (seconds)
	Sim.setTimeStep(0.01f);

	//number of pendulums
	unsigned int n{ 2 };

	//pivot array
	glm::vec3 pivot{ glm::vec3(0.0f,1.0f,0.0f)};

	//length array
	float* length{ new float[2] {0.5f, 0.4f} };

	//initial angle array
	float* initAng{ new float[2] {1.0f, 0.5f} };

	//initial angular velocity array
	float* initAngVel{ new float[2] {0.1f, 0.0f} };

	//nodes array
	unsigned int* nodes{ new unsigned int[2] {10, 10} };

	//setup the pendulum, by adding the initial states
	DataStore data{ Sim.setUpPend(n, pivot, length, initAng, initAngVel, nodes) };

	//initialize the shaders and get the shader program (AFTER window creation!)
	GLuint shaderProgram{ initShaders() };

	//delete the arrays that were used to set up the pendulum
	delete[] length;
	delete[] initAng;
	delete[] initAngVel;
	delete[] nodes;

	GLuint* VBO, *VAO;

	VBO = data.getVBO();
	VAO = data.getVAO();
	GLFWwindow* window{ data.getWindow() };
	GLfloat prevTime{};

	//create the translation matrices for the circles and lines
	glm::mat4* Ctran{ new glm::mat4[n]{1.0f} }, *Ltran{ new glm::mat4[n]{1.0f} };
	int translation;

	
	while (!glfwWindowShouldClose(window)) {
		glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
		glUseProgram(shaderProgram);
		glClear(GL_COLOR_BUFFER_BIT);

		translation = glGetUniformLocation(shaderProgram, "translate");

		for (unsigned int i{}; i < n; i++) {
			glUniformMatrix4fv(translation, 1, GL_FALSE, glm::value_ptr(Ctran[i]));
			glBindVertexArray(VAO[i]);
			glDrawArrays(GL_TRIANGLE_FAN, 0, Sim.getPendNum()[i].getCircleVertices());


			glUniformMatrix4fv(translation, 1, GL_FALSE, glm::value_ptr(Ltran[i]));
			glBindVertexArray(VAO[i]);
			glDrawArrays(GL_LINES, 0, Sim.getPendNum()[i].getLineVertices());
		}

		if (glfwGetTime() - prevTime > 0.01f) {
			Sim.RK4Step();

			//fill the matrices with the new translated matrices
			for (unsigned int i{}; i < n; i++) {
				Ctran[i] = glm::translate(Ctran[i], Sim.getPendNum()[i].getPos() - Sim.getPendNum()[i].getPrevPos());
				Ltran[i] = glm::rotate(Ltran[i], Sim.getPendNum()[i].getAngle() - Sim.getPendNum()[i].getAngPrev(), glm::vec3(0.0, 0.0, 1.0));
				prevTime = glfwGetTime();
			}
		}

		glfwSwapBuffers(window);
		glfwPollEvents();
		glBindVertexArray(0);

	}

	data.glClean();
	delete[] Ctran;
	delete[] Ltran;
	return 0;
}