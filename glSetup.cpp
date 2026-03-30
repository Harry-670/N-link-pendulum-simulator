#include "glSetup.h"


GLFWwindow* glSetupWindow(GLuint windowX, GLuint windowY) {
	glfwInit();

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	GLFWwindow* window{ glfwCreateWindow(800, 800, "Dun dun", nullptr, nullptr) };

	glfwMakeContextCurrent(window);

	gladLoadGL();

	glViewport(0, 0, 800, 800);

	return window;
}

#include "Shaders.h"
#include <iostream>
#include <glad/glad.h>

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
	catch (std::ifstream::failure& e) { std::cout << "don't do it"; }

	fileF.close();

	return fileString;

}

unsigned int initShaders() {

	std::string fragS{ readFile("FragmentShader.txt") };
	const char* fragShader{ fragS.c_str() };
	std::string vertS{ readFile("VertexShader.txt") };
	const char* vertShader{ vertS.c_str() };

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

	glDeleteShader(fragmentShader);
	glDeleteShader(vertexShader);
	return shaderProgram;
}