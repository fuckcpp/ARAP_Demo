#pragma once
#include <igl/opengl/glfw/Viewer.h>
#include <igl/unproject_onto_mesh.h>
#include <igl/arap.h>
#include <igl/readDMAT.h>
#include <igl/writeDMAT.h>
#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>
#include <igl/Timer.h>
#include <igl/colon.h>
#include <igl/opengl/ViewerData.h>

#include <iostream>
#include <stdlib.h>
#include <string>
#include <vector>

using namespace std;
using namespace Eigen;
using namespace igl::opengl::glfw;

enum COORD_TYPE {
	X_COORD,
	Y_COORD,
	Z_COORD,
	NONE_COORD
};

class Model
{
public:
	Model(string name_in):name(name_in) {};
	Model(const Model& other_model)
	{
		color = other_model.color;
		v_const = other_model.v_const;
		v_setting = other_model.v_setting;
		//memmove(&arap_data, &other_model.arap_data, sizeof(arap_data));
		//arap_data = other_model.arap_data;
		name = other_model.name;
	}
	~Model() {};
	MatrixXd color;
	VectorXi v_const, v_setting;
	//igl::ARAPData arap_data;
	string name;
};
