//g2o
#include<g2o/core/sparse_optimizer.h>
#include<g2o/core/block_solver.h>
#include<g2o/core/robust_kernel.h>
#include<g2o/core/robust_kernel_impl.h>
#include<g2o/core/optimization_algorithm_levenberg.h>
#include<g2o/solvers/cholmod/linear_solver_cholmod.h>
//#include<g2o/solvers/csparse/linear_solver_csparse.h>
#include<g2o/types/slam3d/edge_se3.h>
#include<g2o/types/slam3d/vertex_se3.h>

int main()
{
    //构造g2o的图
        //1.求解器
        g2o::SparseOptimizer optimizer;
        //使用Cholmod中的线性求解器
        //g2o::BlockSolver_6_3::LinearSolverType* linearSolver=new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>();
        //6*3参数
       // g2o::BlockSolver_6_3* block_solver=new g2o::BlockSolver_6_3(linearSolver);
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<Eigen::Dynamic,Eigen::Dynamic>> myBlockSlover;
        //typedef g2o::LinearSolverCholmod<myBlockSlover::PoseLandmarkMatrixType> myLinearSolver;
        typedef g2o::LinearSolverCholmod<myBlockSlover::PoseLandmarkMatrixType> myLinearSolver;
        //L-M
        g2o::OptimizationAlgorithmLevenberg* algoritnm=new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<myBlockSlover>(g2o::make_unique<myLinearSolver>()));
        optimizer.setAlgorithm(algoritnm);
        optimizer.setVerbose(false);
}
