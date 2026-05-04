#include "glSetup.h"
#include <iostream>
#include <glad/glad.h>

// Initializes GLFW, requests OpenGL 3.3 core, creates the window and makes
// its context current, then loads the GL function pointers via GLAD
GLFWwindow* DataStore::glSetupWindow(GLuint windowX, GLuint windowY) {
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	GLFWwindow* window{ glfwCreateWindow(windowX, windowY, "N - Pendulum Simulator", nullptr, nullptr) };

	glfwMakeContextCurrent(window);
	gladLoadGL();
	glViewport(0, 0, windowX, windowY);

	this->window = window;
	return window;
}

GLFWwindow* DataStore::getWindow() {
	return window;
}

// Reads the entire file at `file` into a string
// Returns empty on failure
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

// Loads, compiles, and links the vertex and fragment shaders from disk
// Returns the linked program handle (or 0 / unusable handle on failure)
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
		std::cout << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n" << infoLog << std::endl;
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

	// Shaders can be detached once the program is linked
	glDeleteShader(fragmentShader);
	glDeleteShader(vertexShader);
	return shaderProgram;
}

// `size` is the vertex count; the float buffer is laid out as 2 floats per
// vertex (x, y), so the total number of floats is size * 2
unsigned int DataStore::addShape(float* shape, unsigned int size) {
	bufferObjectArrays[index] = new float[size * 2];

	for (unsigned int i{}; i < size * 2; i++) {
		bufferObjectArrays[index][i] = shape[i];
	}

	glBindVertexArray(VAO[index]);
	glBindBuffer(GL_ARRAY_BUFFER, VBO[index]);
	glBufferData(GL_ARRAY_BUFFER, size * 2 * sizeof(GLfloat), bufferObjectArrays[index], GL_DYNAMIC_DRAW);

	// Attribute 0 = vec2 position, tightly packed.
	glVertexAttribPointer(0, 2, GL_FLOAT, false, sizeof(GLfloat) * 2, (void*)0);
	glEnableVertexAttribArray(0);

	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	delete[] shape;

	index++;
	return index - 1;
}

// Allocates the parallel VBO / VAO / vertex-data arrays and asks GL for the
// buffer and vertex-array names. Requires an active GL context
void DataStore::genShapes(unsigned int numShapes) {
	n = numShapes;
	VBO = new GLuint[n];
	VAO = new GLuint[n];
	bufferObjectArrays = new float* [n];

	glGenBuffers(n, VBO);
	glGenVertexArrays(n, VAO);
}

void DataStore::glClean() {
	for (unsigned int i{}; i < n; i++) {
		delete[] bufferObjectArrays[i];
	}

	delete[] bufferObjectArrays;
	delete[] VBO;
	delete[] VAO;

	glfwDestroyWindow(window);
	glfwTerminate();
}