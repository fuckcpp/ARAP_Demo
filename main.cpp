#include "model.h"

COORD_TYPE coord_type = NONE_COORD;


void render_quads_obj(string model, MatrixXd& v_out, MatrixXi& f_out);
bool key_down(Viewer& viewer, unsigned char key, int modifier);
bool key_pressed(Viewer& viewer, unsigned int key, int modifiers);
bool mouse_down(Viewer& viewer, int button, int modifier);
void move_face(Viewer& viewer, COORD_TYPE coord_type, double move_distance);
bool pre_draw(Viewer& viewer);
//void ARAP_initial(Viewer& viewer);
void ARAP_PreCompute(Viewer& viewer);

igl::Timer timer;

MatrixXd color;
int fid;
int lastfid = -1;
igl::ARAPData arap_data;

RowVector3d red(255/ 255.0, 0 / 255.0, 0 / 255.0);
RowVector3d purple(80.0 / 255.0, 64.0 / 255.0, 255.0 / 255.0);
RowVector3d gold(255.0 / 255.0, 228.0 / 255.0, 58.0 / 255.0);
bool bSetting = false;


//string model = "/gaoxin-selection";
//string model = "/box_test";
//string model = "/cube_40k";

vector<string> names =
{/*"/NiGaoXinII_Base-Low-Head","/horse_quad", */"/NiGaoXinII_Base-Low-Head","/eye"/*,"/box_test"*/};
vector<Model> models;

int select_index = 0;

int main(int argc, char* argv[]) {

	Viewer viewer;
	std::map<int, Eigen::RowVector3d> colors;
	for (int i=0;i<names.size();i++)
	{
		select_index = i;
		Model model(names[select_index]);
		models.push_back(model);
		MatrixXd v_tri; MatrixXi f_tri;
		render_quads_obj(model.name, v_tri, f_tri);

		if (!(viewer.data().F.rows() == 0 && viewer.data().V.rows() == 0))
		{
			viewer.append_mesh();
		}
		viewer.data().clear();
		viewer.data().set_mesh(v_tri, f_tri);
		colors.emplace(viewer.data().id, 0.5 * Eigen::RowVector3d::Random().array() + 0.5);

		igl::opengl::ViewerData& SelectedData = viewer.data_list[select_index];
		VectorXi& v_setting = models[select_index].v_setting;
		v_setting.resize(SelectedData.V.rows());
		v_setting.setConstant(-1);
		igl::readDMAT(TUTORIAL_SHARED_PATH + models[select_index].name + ".dmat", v_setting);
		models[select_index].color = Eigen::MatrixXd::Constant(SelectedData.F.rows(), 3, 1);
		for (int f = 0; f < SelectedData.F.rows(); f++)
		{
			if (v_setting(SelectedData.F(f, 0)) >= 0 && v_setting(SelectedData.F(f, 1)) >= 0 && v_setting(SelectedData.F(f, 2)) >= 0)
			{
				models[select_index].color.row(f) = purple;
			}
			else
			{
				models[select_index].color.row(f) = gold;
			}
		}
		arap_data.max_iter = 10;
		ARAP_PreCompute(viewer);
	}

	//ARAP_initial(viewer);

	//viewer.core().align_camera_center(v_tri, f_tri);
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

//void ARAP_initial(Viewer& viewer)
//{
//	igl::opengl::ViewerData& SelectedData = viewer.data_list[select_index];
//	v_setting.resize(SelectedData.V.rows());
//	v_setting.setConstant(-1);
//	igl::readDMAT(TUTORIAL_SHARED_PATH +model+".dmat", v_setting);
//	color = Eigen::MatrixXd::Constant(SelectedData.F.rows(), 3, 1);
//	for (int f = 0; f < SelectedData.F.rows(); f++)
//	{
//		if (v_setting(SelectedData.F(f, 0)) >= 0 && v_setting(SelectedData.F(f, 1)) >= 0 && v_setting(SelectedData.F(f, 2)) >= 0)
//		{
//			color.row(f) = purple;
//		}
//		else
//		{
//			color.row(f) = gold;
//		}
//	}
//	arap_data.max_iter = 10;
//
//	ARAP_PreCompute(viewer);
//}

void ARAP_PreCompute(Viewer& viewer)
{
	igl::opengl::ViewerData& SelectedData = viewer.data_list[select_index];
	VectorXi& v_const = models[select_index].v_const;
	VectorXi& v_setting = models[select_index].v_setting;
	igl::colon<int>(0, SelectedData.V.rows() - 1, v_const);
	v_const.conservativeResize(stable_partition(v_const.data(), v_const.data() + v_const.size(),
		[&v_setting](int i)->bool {return v_setting(i) >= 0; }) - v_const.data());
	igl::arap_precomputation(SelectedData.V, SelectedData.F, SelectedData.V.cols(), v_const, arap_data);
}

bool pre_draw(Viewer& viewer)
{
	MatrixXd V_select;
	igl::opengl::ViewerData& SelectedData = viewer.data_list[select_index];
	VectorXi& v_const = models[select_index].v_const;
	V_select.resize(v_const.rows(), SelectedData.V.cols());

	for (int i = 0; i < v_const.size(); i++)
	{
		V_select.row(i) = SelectedData.V.row(v_const(i));
	}

	if (V_select.size() > 0)
	{
		igl::arap_solve(V_select, arap_data, SelectedData.V);
		viewer.data().set_vertices(SelectedData.V);
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
				VectorXi& v_setting = models[select_index].v_setting;
				cout << "save v_setting:" << v_setting << endl;
				MatrixXi temp(v_setting);
				igl::writeDMAT(TUTORIAL_SHARED_PATH + models[select_index].name + ".dmat", temp, true);
				ARAP_PreCompute(viewer);
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
	igl::opengl::ViewerData& SelectedData = viewer.data_list[select_index];
	if (fid > 0)
	{
		switch (coord_type)
		{
		case X_COORD: {
			SelectedData.V.row(SelectedData.F.row(fid)[0])[0] += move_distance;
			SelectedData.V.row(SelectedData.F.row(fid)[1])[0] += move_distance;
			SelectedData.V.row(SelectedData.F.row(fid)[2])[0] += move_distance;
			break;
		}
		case Z_COORD: {
			SelectedData.V.row(SelectedData.F.row(fid)[0])[1] += move_distance;
			SelectedData.V.row(SelectedData.F.row(fid)[1])[1] += move_distance;
			SelectedData.V.row(SelectedData.F.row(fid)[2])[1] += move_distance;
			break;
		}
		case Y_COORD: {
			SelectedData.V.row(SelectedData.F.row(fid)[0])[2] += move_distance;
			SelectedData.V.row(SelectedData.F.row(fid)[1])[2] += move_distance;
			SelectedData.V.row(SelectedData.F.row(fid)[2])[2] += move_distance;
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
	igl::opengl::ViewerData& SelectedData=viewer.data_list[select_index];
	VectorXi& v_setting = models[select_index].v_setting;

	if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), viewer.core().view,
		viewer.core().proj, viewer.core().viewport, SelectedData.V, SelectedData.F, fid, bc))
	{
		v_setting(SelectedData.F(fid, 0)) += 1;
		v_setting(SelectedData.F(fid, 1)) += 1;
		v_setting(SelectedData.F(fid, 2)) += 1;

		if (bSetting)
		{
			models[select_index].color.row(fid) << purple;
			//viewer.data().set_colors(color);
			SelectedData.set_colors(models[select_index].color);
			return true;
		}
		else
		{
			models[select_index].color.row(fid) << red;
			if (lastfid >= 0)
			{
				v_setting(SelectedData.F(lastfid, 0)) -= 1;
				v_setting(SelectedData.F(lastfid, 1)) -= 1;
				v_setting(SelectedData.F(lastfid, 2)) -= 1;
				models[select_index].color.row(lastfid) << gold;
			}
			//viewer.data().set_colors(color);
			SelectedData.set_colors(models[select_index].color);
			lastfid = fid;

			ARAP_PreCompute(viewer);
			return true;
		}

	}
	return false;
}

void render_quads_obj(string model, MatrixXd& v_out, MatrixXi& f_out) {
	MatrixXd V;
	MatrixXi F;
	timer.start();
	igl::readOBJ(TUTORIAL_SHARED_PATH + model + ".obj", V, F);

	cout << "load obj time = " << timer.getElapsedTime() << endl;
	v_out = V;
	f_out.resize(F.rows() * 2, 3);
	for (int i = 0; i < F.rows(); i++)
	{
		f_out.row(i * 2 + 0) << F.row(i)[0], F.row(i)[1], F.row(i)[2];
		f_out.row(i * 2 + 1) << F.row(i)[0], F.row(i)[2], F.row(i)[3];
	}
}