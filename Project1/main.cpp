#include <igl/read_triangle_mesh.h>
#include <igl/principal_curvature.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/per_vertex_normals.h>
#include <Eigen/Core>
#include <iostream>
#include <igl/jet.h>

int main(int argc, char *argv[])
{
    // 1. ��������
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    if (!igl::read_triangle_mesh((argc > 1) ? argv[1] : "D:/MyDocs/Models/input/bunny.obj", V, F))
    {
        std::cerr << "Failed to load mesh." << std::endl;
        return -1;
    }

    // 2. ����������
    Eigen::MatrixXd PD1, PD2;   // �����򣨵�λ������
    Eigen::VectorXd PV1, PV2;   // ������ֵ��������

    // ��ѡ������number of iterations for smoothing (Ĭ��5)
    igl::principal_curvature(V, F, PD1, PD2, PV1, PV2, 5);

    //// ��ѡ�����㶥�㷨�����ڿ��ӻ�
    //Eigen::MatrixXd N;
    //igl::per_vertex_normals(V, F, N);


    Eigen::VectorXd K = PV1.array() * PV2.array();//K = k1*k2 ��˹����
    Eigen::MatrixXd C;
    //igl::jet(PV1, true, C); // ����ɫ��ʾ��������ʣ�PV1��
    igl::jet(K,true,C);

    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V, F);
    viewer.data().set_colors(C);

    //viewer.data().set_normals(N);

  

    //// ��ѡ����ʾ��������������Ϊ�߶Σ�
    //const double scale = 0.01; // �������ӣ�����ģ�ʹ�С����
    //Eigen::MatrixXd P_start = V;
    //Eigen::MatrixXd P_end1 = V + scale * PD1;
    //Eigen::MatrixXd P_end2 = V + scale * PD2;

    //// ��ӷ���1���߶�
    //for (int i = 0; i < V.rows(); ++i)
    //{
    //    viewer.data().add_edges(P_start.row(i), P_end1.row(i), Eigen::RowVector3d(1, 0, 0)); // ��ɫ
    //    viewer.data().add_edges(P_start.row(i), P_end2.row(i), Eigen::RowVector3d(0, 1, 0)); // ��ɫ
    //}

    viewer.launch();
    return 0;
}