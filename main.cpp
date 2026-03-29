#include <glad/glad.h>
#include < GLFW/glfw3.h >
#include <iostream>
#include "shaders.h"
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "Physics.h"
#include "glSetup.h"

int main() {

	float nodes{ 10 }, initAngle{ 1.0f }, ballRadius{ 0.1 }, screenX{ 800 }, screenY{ 800 }, length(0.5f);

	GLFWwindow* window{ glSetupWindow(screenX, screenY) };

	//init ball 1
	Particle ball{};

	ball.pivot = glm::vec3(0.0f, 0.5f, 0.0f);

	ball.setNodes(nodes);

	ball.setAngle(initAngle);

	ball.setAngVel(0.1f);

	ball.setRadius(ballRadius);

	ball.setLength(length);


	ball.pos = ball.polarToCartVert();

	float* vertices{ ball.initCircle()};

	GLuint VBO, VAO;

	
	glGenBuffers(1, &VBO);

	glGenVertexArrays(1, &VAO);

	glBindVertexArray(VAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 2 * (ball.getNodes() + 2), vertices, GL_DYNAMIC_DRAW);

	glVertexAttribPointer(0, 2, GL_FLOAT, false, sizeof(GLfloat) * 2, (void*)0);
	glEnableVertexAttribArray(0);


	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

		GLuint shaderProgram{ initShaders() };

	glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);
	glfwSwapBuffers(window);
	glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);
	
	GLfloat prevTime{};
	glm::mat4 translate{ 1.0f };
	
	translate = glm::translate(translate, ball.pos);
	int translation{ glGetUniformLocation(shaderProgram, "translate") };
	glUniformMatrix4fv(translation, 1, GL_FALSE, glm::value_ptr(translate));

	while (!glfwWindowShouldClose(window)) {

		glUseProgram(shaderProgram);
		glBindVertexArray(VAO);
		
		glClear(GL_COLOR_BUFFER_BIT);
		glDrawArrays(GL_TRIANGLE_FAN, 0, ball.getNodes() +2);
		glDrawArrays(GL_LINES, ball.getNodes() + 2, 2);
		// got to get it organized so I don't have to do this


		if (glfwGetTime() - prevTime > 0.01f) {

			ball.posPrev = ball.pos;
			ball.newStep(0.01f) ;
			
			translate = glm::translate(translate, ball.pos - ball.posPrev);
			translation = glGetUniformLocation(shaderProgram, "translate");
			glUniformMatrix4fv( translation, 1, GL_FALSE, glm::value_ptr(translate));
			prevTime = glfwGetTime();

		}

		glfwSwapBuffers(window);
		glfwPollEvents();
		glBindVertexArray(0);

	}
	delete[] vertices;
	vertices = nullptr;
	glfwDestroyWindow(window);
	glfwTerminate();
	return 0;
}