#include <igl/opengl/glfw/Viewer.h>
#include <igl/unproject_onto_mesh.h>
#include <igl/arap.h>
#include <igl/readDMAT.h>
#include <igl/writeDMAT.h>
#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>
#include <igl/Timer.h>
#include <igl/colon.h>

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
COORD_TYPE coord_type = NONE_COORD;


void render_quads_obj(Viewer& viewer);
bool key_down(Viewer& viewer, unsigned char key, int modifier);
bool key_pressed(Viewer& viewer, unsigned int key, int modifiers);
bool mouse_down(Viewer& viewer, int button, int modifier);
void move_face(Viewer& viewer, COORD_TYPE coord_type, double move_distance);
bool pre_draw(Viewer& viewer);
void print_MatrixXd(const MatrixXd& matrix);
void print_MatrixXi(const MatrixXi& matrix);
void ARAP_initial();
void ARAP_PreCompute();

igl::Timer timer;

MatrixXd v_tri; MatrixXi f_tri;
MatrixXd color;
int fid;
int lastfid = -1;
igl::ARAPData arap_data;
VectorXi v_const,v_setting;

RowVector3d red(255/ 255.0, 0 / 255.0, 0 / 255.0);
RowVector3d purple(80.0 / 255.0, 64.0 / 255.0, 255.0 / 255.0);
RowVector3d gold(255.0 / 255.0, 228.0 / 255.0, 58.0 / 255.0);
bool bSetting = false;

string model = "/horse_quad";
//string model = "/gaoxin-selection";
//string model = "/box_test";
//string model = "/cube_40k";

int main(int argc, char* argv[]) {

	Viewer viewer;

	render_quads_obj(viewer);
	ARAP_initial();

	viewer.data().set_mesh(v_tri, f_tri);
	viewer.core().align_camera_center(v_tri, f_tri);
	viewer.data().set_colors(color);
	viewer.data().set_face_based(true);
	viewer.data().show_lines = true;
	viewer.data().show_texture = false;

	viewer.callback_key_down = key_down;
	viewer.callback_key_pressed= key_pressed;
	viewer.callback_mouse_down = mouse_down;
	viewer.callback_pre_draw= pre_draw;
	viewer.launch();
	return 0;
}

void ARAP_initial()
{
	v_setting.resize(v_tri.rows());
	v_setting.setConstant(-1);
	igl::readDMAT(TUTORIAL_SHARED_PATH +model+".dmat", v_setting);
	color = Eigen::MatrixXd::Constant(f_tri.rows(), 3, 1);
	for (int f = 0; f < f_tri.rows(); f++)
	{
		if (v_setting(f_tri(f, 0)) >= 0 && v_setting(f_tri(f, 1)) >= 0 && v_setting(f_tri(f, 2)) >= 0)
		{
			color.row(f) = purple;
		}
		else
		{
			color.row(f) = gold;
		}
	}
	arap_data.max_iter = 10;

	ARAP_PreCompute();
}

void ARAP_PreCompute()
{
	igl::colon<int>(0, v_tri.rows() - 1, v_const);
	v_const.conservativeResize(stable_partition(v_const.data(), v_const.data() + v_const.size(),
		[](int i)->bool {return v_setting(i) >= 0; }) - v_const.data());
	igl::arap_precomputation(v_tri, f_tri, v_tri.cols(), v_const, arap_data);
}

void print_MatrixXd(const MatrixXd& matrix)
{
	for (int i = 0; i < matrix.rows(); i++)
	{
		cout << matrix.row(i) << endl;;
	}
	cout << endl;
}
void print_MatrixXi(const MatrixXi& matrix)
{
	for (int i = 0; i < matrix.rows(); i++)
	{
		cout << matrix.row(i) << endl;;
	}
	cout << endl;
}
bool pre_draw(Viewer& viewer)
{
	MatrixXd V_select;
	V_select.resize(v_const.rows(), v_tri.cols());

	for (int i = 0; i < v_const.size(); i++)
	{
		V_select.row(i) = v_tri.row(v_const(i));
	}

	if (V_select.size() > 0)
	{
		igl::arap_solve(V_select, arap_data, v_tri);
		viewer.data().set_vertices(v_tri);
		viewer.data().compute_normals();
	}
	return false;
}

bool key_down(Viewer& viewer, unsigned char key, int modifier) {
		switch (key) {
		case 'C':
		case 'c': {
			if (bSetting)
			{
				cout << "save v_setting:" << v_setting << endl;
				MatrixXi temp(v_setting);
				igl::writeDMAT(TUTORIAL_SHARED_PATH + model + ".dmat", temp, true);
				ARAP_PreCompute();
			}
			bSetting = !bSetting;
		}
				 
		default:
			break;
		}
	return false;
}

bool key_pressed(Viewer& viewer, unsigned int key, int modifiers)
{
	double distance = 0.01;
		switch (key) {
		case 'A':
		case 'a': {
			coord_type = X_COORD;
			cout << "coord:" << coord_type<< "move:" << -distance << endl;
			move_face(viewer, coord_type ,-distance);
			break;
		}
		case 'D':
		case 'd': {
			coord_type = X_COORD;
			move_face(viewer, coord_type, distance);
			cout << "coord:" << coord_type << "move:" << distance << endl;
			break;
		}
		case 'W':
		case 'w': {
			coord_type = Z_COORD;
			cout << "coord:" << coord_type << "move:" << distance << endl;
			move_face(viewer, coord_type, distance);
			break;
		}
		case 'S':
		case 's': {
			coord_type = Z_COORD;
			move_face(viewer, coord_type, -distance);
			cout << "coord:" << coord_type << "move:" << -distance << endl;
			break;
		}
		case 'Z':
		case 'z': {
			coord_type = Y_COORD;
			cout << "coord:" << coord_type << "move:" << distance << endl;
			move_face(viewer, coord_type, distance);
			break;
		}
		case 'E':
		case 'e': {
			coord_type = Y_COORD;
			move_face(viewer, coord_type, -distance);
			cout << "coord:" << coord_type << "move:" << -distance << endl;
			break;
		}
		default:
			break;
		}
	return false;
}

void move_face(Viewer& viewer, COORD_TYPE coord_type, double move_distance)
{
	if (fid > 0)
	{
		switch (coord_type)
		{
		case X_COORD: {
			v_tri.row(f_tri.row(fid)[0])[0] += move_distance;
			v_tri.row(f_tri.row(fid)[1])[0] += move_distance;
			v_tri.row(f_tri.row(fid)[2])[0] += move_distance;
			break;
		}
		case Z_COORD: {
			v_tri.row(f_tri.row(fid)[0])[1] += move_distance;
			v_tri.row(f_tri.row(fid)[1])[1] += move_distance;
			v_tri.row(f_tri.row(fid)[2])[1] += move_distance;
			break;
		}
		case Y_COORD: {
			v_tri.row(f_tri.row(fid)[0])[2] += move_distance;
			v_tri.row(f_tri.row(fid)[1])[2] += move_distance;
			v_tri.row(f_tri.row(fid)[2])[2] += move_distance;
			break;
		}
		default:
			break;
		}

	}
}

bool mouse_down(Viewer& viewer, int button, int modifier)
{
	Eigen::Vector3f bc;
	// Cast a ray in the view direction starting from the mouse position
	double x = viewer.current_mouse_x;
	double y = viewer.core().viewport(3) - viewer.current_mouse_y;
	if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), viewer.core().view,
		viewer.core().proj, viewer.core().viewport, v_tri, f_tri, fid, bc))
	{
		v_setting(f_tri(fid, 0)) += 1;
		v_setting(f_tri(fid, 1)) += 1;
		v_setting(f_tri(fid, 2)) += 1;

		if (bSetting)
		{
			color.row(fid) << purple;
			viewer.data().set_colors(color);
			return true;
		}
		else
		{
			color.row(fid) << red;
			if (lastfid >= 0)
			{
				v_setting(f_tri(lastfid, 0)) -= 1;
				v_setting(f_tri(lastfid, 1)) -= 1;
				v_setting(f_tri(lastfid, 2)) -= 1;
				color.row(lastfid) << gold;
			}
			viewer.data().set_colors(color);
			lastfid = fid;

			ARAP_PreCompute();
			return true;
		}

	}
	return false;
}

void render_quads_obj(Viewer& viewer) {
	MatrixXd V;
	MatrixXi F;
	timer.start();
	igl::readOBJ(TUTORIAL_SHARED_PATH + model + ".obj", V, F);

	cout << "load obj time = " << timer.getElapsedTime() << endl;
	v_tri = V;
	f_tri.resize(F.rows() * 2, 3);
	for (int i = 0; i < F.rows(); i++)
	{
		f_tri.row(i * 2 + 0) << F.row(i)[0], F.row(i)[1], F.row(i)[2];
		f_tri.row(i * 2 + 1) << F.row(i)[0], F.row(i)[2], F.row(i)[3];
	}
}