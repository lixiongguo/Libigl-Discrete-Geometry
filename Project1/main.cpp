#include <igl/read_triangle_mesh.h>
#include <igl/principal_curvature.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/per_vertex_normals.h>
#include <Eigen/Core>
#include <iostream>
#include <igl/jet.h>

int main(int argc, char *argv[])
{
    // 1. 加载网格
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    if (!igl::read_triangle_mesh((argc > 1) ? argv[1] : "D:/MyDocs/Models/input/bunny.obj", V, F))
    {
        std::cerr << "Failed to load mesh." << std::endl;
        return -1;
    }

    // 2. 计算主曲率
    Eigen::MatrixXd PD1, PD2;   // 主方向（单位向量）
    Eigen::VectorXd PV1, PV2;   // 主曲率值（标量）

    // 可选参数：number of iterations for smoothing (默认5)
    igl::principal_curvature(V, F, PD1, PD2, PV1, PV2, 5);

    //// 可选：计算顶点法向用于可视化
    //Eigen::MatrixXd N;
    //igl::per_vertex_normals(V, F, N);


    Eigen::VectorXd K = PV1.array() * PV2.array();//K = k1*k2 高斯曲率
    Eigen::MatrixXd C;
    //igl::jet(PV1, true, C); // 用颜色显示最大主曲率（PV1）
    igl::jet(K,true,C);

    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V, F);
    viewer.data().set_colors(C);

    //viewer.data().set_normals(N);

  

    //// 可选：显示主方向向量（作为线段）
    //const double scale = 0.01; // 缩放因子，根据模型大小调整
    //Eigen::MatrixXd P_start = V;
    //Eigen::MatrixXd P_end1 = V + scale * PD1;
    //Eigen::MatrixXd P_end2 = V + scale * PD2;

    //// 添加方向1的线段
    //for (int i = 0; i < V.rows(); ++i)
    //{
    //    viewer.data().add_edges(P_start.row(i), P_end1.row(i), Eigen::RowVector3d(1, 0, 0)); // 红色
    //    viewer.data().add_edges(P_start.row(i), P_end2.row(i), Eigen::RowVector3d(0, 1, 0)); // 绿色
    //}

    viewer.launch();
    return 0;
}