#include <igl/arap.h>
#include <igl/boundary_loop.h>
#include <igl/harmonic.h>
#include <igl/map_vertices_to_circle.h>
//#include <igl/readOFF.h>
#include <igl/readOBJ.h>
#include <igl/opengl/glfw/Viewer.h>

#define TUTORIAL_SHARED_PATH "D:\\GeometryLab\\Lab\\models\\libigl_models\\"

Eigen::MatrixXd V;
Eigen::MatrixXi F;
Eigen::MatrixXd V_uv;
Eigen::MatrixXd initial_guess;

bool show_uv = false;

bool key_down(igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifier)
{
    if (key == '1')
        show_uv = false;
    else if (key == '2')
        show_uv = true;

    if (key == 'q')
        V_uv = initial_guess;

    if (show_uv)
    {
        viewer.data().set_mesh(V_uv, F);
        viewer.core().align_camera_center(V_uv, F);
    }
    else
    {
        viewer.data().set_mesh(V, F);
        viewer.core().align_camera_center(V, F);
        //viewer.data().show_texture = false;
    }

    viewer.data().compute_normals();

    return false;
}


int main(int argc, char* argv[])
{
    using namespace std;
    // Load a mesh in OFF format
    //Cat_head.obj /David328.obj Bunny_head.obj
    igl::readOBJ(TUTORIAL_SHARED_PATH "/Bunny_head.obj", V, F);

    // Compute the initial solution for ARAP (harmonic parametrization)
    Eigen::VectorXi bnd;
    igl::boundary_loop(F, bnd);

    Eigen::MatrixXd bnd_uv;
    igl::map_vertices_to_circle(V, bnd, bnd_uv);

    //    // Add dynamic regularization to avoid to specify boundary conditions
    //igl::ARAPData arap_data;
    //arap_data.with_dynamics = true;
    //    // Initialize ARAP
    //arap_data.max_iter = 100;
  
    // Eigen::VectorXi b = Eigen::VectorXi::Zero(0);
    //Eigen::MatrixXd bc = Eigen::MatrixXd::Zero(0, 0);
    //  // 2 means that we're going to *solve* in 2d
    //arap_precomputation(V, F, 2, b, arap_data);

    igl::harmonic(V, F, bnd, bnd_uv, 1, initial_guess);
    V_uv = initial_guess;

    //arap_solve(bc, arap_data, V_uv);

    V_uv *= 20;
        // Plot the mesh
    igl::opengl::glfw::Viewer viewer;
    viewer.callback_key_down =&key_down;

    viewer.data().set_mesh(V, F);
    viewer.data().set_uv(V_uv);

        // Draw checkerboard texture
    viewer.data().show_texture = true;

    // Launch the viewer
    viewer.launch();

}