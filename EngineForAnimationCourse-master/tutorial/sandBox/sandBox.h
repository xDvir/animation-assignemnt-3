#pragma once
#include "igl/opengl/glfw/Viewer.h"
#include "igl/aabb.h"

using namespace std;
using namespace Eigen;
using namespace igl;

class SandBox : public igl::opengl::glfw::Viewer
{
	
public:
	bool alreadyCollapse = false;
	SandBox();
	~SandBox();

	void Init(const std::string& config);
	double doubleVariable;
private:
	// Prepare array-based edge data structures and priority queue
	
	
	void Animate();
};

