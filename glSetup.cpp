#include "glSetup.h"
#include "Shaders.h"
#include <iostream>
#include <glad/glad.h>

GLFWwindow* DataStore::glSetupWindow(GLuint windowX, GLuint windowY) {
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	GLFWwindow* window{ glfwCreateWindow(800, 800, "Dun dun", nullptr, nullptr) };

	glfwMakeContextCurrent(window);
	gladLoadGL();
	glViewport(0, 0, 800, 800);

	this->window = window;
	return window;
}

GLFWwindow* DataStore::getWindow() {
	return window;
}

std::string readFile(const char* file) {
	std::fstream fileF;
	std::stringstream fileS;
	std::string fileString;

	fileF.exceptions(std::fstream::failbit | std::fstream::badbit);
	try {
		fileF.open(file, std::ios::in);
		fileS << fileF.rdbuf();
		fileString = fileS.str();
	}
	catch (std::ifstream::failure&) {
		std::cout << "ERROR: Failed to open file: " << file << std::endl;
	}

	fileF.close();
	return fileString;
}

unsigned int initShaders() {
	std::string fragS{ readFile("FragmentShader.txt") };
	const char* fragShader{ fragS.c_str() };
	std::string vertS{ readFile("VertexShader.txt") };
	const char* vertShader{ vertS.c_str() };

	if (fragS.empty()) {
		std::cout << "ERROR: FragmentShader.txt not found or empty!" << std::endl;
	}
	if (vertS.empty()) {
		std::cout << "ERROR: VertexShader.txt not found or empty!" << std::endl;
	}

	int success;
	char infoLog[512];

	GLuint fragmentShader{ glCreateShader(GL_FRAGMENT_SHADER) };
	glShaderSource(fragmentShader, 1, &fragShader, nullptr);
	glCompileShader(fragmentShader);

	glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
	if (!success) {
		glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
		std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << std::endl;
	}

	GLuint vertexShader{ glCreateShader(GL_VERTEX_SHADER) };
	glShaderSource(vertexShader, 1, &vertShader, nullptr);
	glCompileShader(vertexShader);

	glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
	if (!success) {
		glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
		std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << std::endl;
	}

	GLuint shaderProgram{ glCreateProgram() };
	glAttachShader(shaderProgram, fragmentShader);
	glAttachShader(shaderProgram, vertexShader);
	glLinkProgram(shaderProgram);

	glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
	if (!success) {
		glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
		std::cout << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
	}

	glDeleteShader(fragmentShader);
	glDeleteShader(vertexShader);
	return shaderProgram;
}

unsigned int DataStore::addShape(float* &shape, unsigned int size) {
	BOArrays[index] = new float[size * 2];

	for (unsigned int i{}; i < size * 2; i++) {
		BOArrays[index][i] = shape[i];
	}

	glBindVertexArray(VAO[index]);
	glBindBuffer(GL_ARRAY_BUFFER, VBO[index]);
	glBufferData(GL_ARRAY_BUFFER, size * 2 * sizeof(GLfloat), BOArrays[index], GL_DYNAMIC_DRAW);

	glVertexAttribPointer(0, 2, GL_FLOAT, false, sizeof(GLfloat) * 2, (void*)0);
	glEnableVertexAttribArray(0);

	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	delete[] shape;
	shape = nullptr;

	index++;
	return index - 1;
}

void DataStore::genShapes(unsigned int numShapes) {
	n = numShapes;
	VBO = new GLuint[n];
	VAO = new GLuint[n];
	BOArrays = new float*[n];

	glGenBuffers(n, VBO);
	glGenVertexArrays(n, VAO);
}

void DataStore::glClean() {
	for (unsigned int i{}; i < n; i++) {
		delete[] BOArrays[i];
	}

	delete[] BOArrays;
	delete[] VBO;
	delete[] VAO;

	glfwDestroyWindow(window);
	glfwTerminate();
}