#pragma once
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <fstream>
#include <sstream>
#include <vector>

std::string readFile(const char* file);

unsigned int initShaders();
//setup the window and viewport as well as initialize the version of OpenGL
GLFWwindow* glSetupWindow(GLuint, GLuint );

class DataStore {

private:

	std::vector<float> vertices;
	unsigned int first;

public:

	DataStore():first(0) {}

	unsigned int addShape(float* &shape, unsigned int size) {
	
		unsigned int ret{ first };
	
		for (unsigned int i{}; i < size*2 ; i++) {

			vertices.push_back(shape[i]);

		}

		first += size;

		delete[] shape;

		shape = nullptr;
	
		return ret;
	}

	void sendToGPU(GLuint &VBO, GLuint &VAO) {

		glGenBuffers(1, &VBO);

		glGenVertexArrays(1, &VAO);

		glBindVertexArray(VAO);

		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER, 4*sizeof(vertices), vertices.data(), GL_DYNAMIC_DRAW);

		glVertexAttribPointer(0, 2, GL_FLOAT, false, sizeof(GLfloat) * 2, (void*)0);
		glEnableVertexAttribArray(0);


		glBindVertexArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);


	}


};