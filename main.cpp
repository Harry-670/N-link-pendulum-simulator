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

	//init ball 1
	Particle ball{};
	DataStore data{};

	GLFWwindow* window{ data.glSetupWindow(screenX, screenY) };

	data.genShapes(2);

	GLuint* VBO, *VAO;

	VBO = data.getVBO();
	VAO = data.getVAO();

	ball.pivot = glm::vec3(0.0f, 0.0f, 0.0f);

	ball.setNodes(nodes);

	ball.setAngle(initAngle);

	ball.setAngVel(0.1f);

	ball.setRadius(ballRadius);

	ball.setLength(length);


	ball.pos = ball.polarToCartVert();

	float *circle{}, *line{ new float [4]{ball.pivot.x,ball.pivot.y,ball.pos.x,ball.pos.y} };
	
	unsigned int sizeC{ ball.initCircle(circle) }, sizeL{2};

	unsigned int firstCircle{ data.addShape(circle, sizeC) };
	unsigned int firstLine{ data.addShape(line, sizeL) };


	GLuint shaderProgram{ initShaders() };

	glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);
	glfwSwapBuffers(window);
	glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);
	
	GLfloat prevTime{};
	glm::mat4 Ctran{ 1.0f }, Ltran{ 1.0f };
	
	Ctran = glm::translate(Ctran, ball.pos);
	int translation{ glGetUniformLocation(shaderProgram, "translate") };
	glUniformMatrix4fv(translation, 1, GL_FALSE, glm::value_ptr(Ctran));

	while (!glfwWindowShouldClose(window)) {

		// Inside your while loop
		glUseProgram(shaderProgram);
		glClear(GL_COLOR_BUFFER_BIT);
		translation = glGetUniformLocation(shaderProgram, "translate");

		// 1. Draw the Ball (Use the 'translate' matrix)
		glUniformMatrix4fv(translation, 1, GL_FALSE, glm::value_ptr(Ctran));
		glBindVertexArray(VAO[firstCircle]);
		glDrawArrays(GL_TRIANGLE_FAN, 0, sizeC);

		// 2. Draw the Line (Use an Identity matrix so it stays anchored)
		
		glUniformMatrix4fv(translation, 1, GL_FALSE, glm::value_ptr(Ltran));
		glBindVertexArray(VAO[firstLine]);
		glDrawArrays(GL_LINES, 0, sizeL);


		if (glfwGetTime() - prevTime > 0.01f) {

			ball.posPrev = ball.pos;
			ball.angPrev = ball.angle;
			ball.newStep(0.01f) ;
			
			//setup the correct getters and setters for physics.cpp
			Ctran = glm::translate(Ctran, ball.pos - ball.posPrev);
			Ltran = glm::rotate(Ltran, ball.angle-ball.angPrev, glm::vec3(0.0, 0.0, 1.0));
			prevTime = glfwGetTime();

		}

		glfwSwapBuffers(window);
		glfwPollEvents();
		glBindVertexArray(0);

	}

	data.glClean();
	
	return 0;
}