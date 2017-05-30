#include <normals.hpp>
#include <chrono>
std::vector<double> estimate_nomal(std::vector<std::vector<double>> points){

    //make sure there are at least 3 points
    if(points.size() < 3){
        return std::vector<double>{-1000,-1000,-1000};
    }

    //find the mean
    double xmean = 0;
    double ymean = 0;
    double zmean = 0;

    for (std::vector<double> d : points){
        xmean += d[0];
        ymean += d[1];
        zmean += d[2];
    }
    xmean /= (double)points.size();
    ymean /= (double)points.size();
    zmean /= (double)points.size();

    //create matrices of demeaned data
    Eigen::Matrix<double, Eigen::Dynamic, 3> demeanData;
    demeanData.resize(points.size(), 3);

    //fill the matrix with the dmeaned data
    for (size_t i = 0; i < points.size(); i++){
        demeanData(i, 0) = points[i][0] - xmean;
        demeanData(i, 1) = points[i][1] - ymean;
        demeanData(i, 2) = points[i][2] - zmean;
    }

    //create the covariance matrix
    Eigen::Matrix<double, 3, 3> covMatrix;
    covMatrix = (1 / ((double)points.size() - 1)) * (demeanData.transpose() * demeanData);

    //compute eigen vectors
    Eigen::EigenSolver<Eigen::Matrix3d> esolve;
    esolve.compute(covMatrix, true);

    Eigen::Matrix<double,3,1> evalAbs = esolve.eigenvalues().cwiseAbs();
    int minInd;
    evalAbs.minCoeff(&minInd);
    std::vector<double> normal{esolve.eigenvectors()(0, minInd).real(),
                               esolve.eigenvectors()(1, minInd).real(),
                               esolve.eigenvectors()(2, minInd).real()};

    //check that this is correct wrt the viewpoint(0,0,0)
    if(normal[0] + normal[1] + normal[2] < 0){
        normal[0] = -normal[0];
        normal[1] = -normal[1];
        normal[2] = -normal[2];
    }
    return normal;
}




std::vector<std::vector<double>> compute_normals(std::vector<std::vector<double>> point_cloud, int max_neighbors, double max_distance) {


    //create a vector of vectors that store the nearest X points
    //up to max_neighbors for each point in the cloud
    std::vector<std::vector<std::vector<double>>> closest_points;

    //create the kd-tree from the point cloud
    typedef KDTreeVectorOfVectorsAdaptor< std::vector<std::vector<double>>, double > kd_tree_t;
    kd_tree_t kd_tree(3, point_cloud, 10);
    kd_tree.index->buildIndex();

    auto t1 = std::chrono::high_resolution_clock::now();
    for (int j = 0; j < point_cloud.size(); ++j) {
        std::vector<size_t>   ret_indexes(max_neighbors);
        std::vector<double> out_dists_sqr(max_neighbors);

        nanoflann::KNNResultSet<double> resultSet(max_neighbors);

        resultSet.init(&ret_indexes[0], &out_dists_sqr[0] );
        kd_tree.index->findNeighbors(resultSet, &point_cloud[j][0], nanoflann::SearchParams(10));

        std::vector<std::vector<double>> points_for_plane;
        //add this point
        points_for_plane.push_back(point_cloud[j]);

        //loop through the points and add ones that are below the max distance to the vector of closest points
        for (int i = 0; i < ret_indexes.size(); ++i) {
            if(sqrt(out_dists_sqr[i]) <= max_distance){
                points_for_plane.push_back(point_cloud[ret_indexes[i]]);
            }
        }
        closest_points.push_back(points_for_plane);
    }
    auto t2 = std::chrono::high_resolution_clock::now();
    //fit a plane to all the points in the closest_points vector
    t1 = std::chrono::high_resolution_clock::now();
    std::vector<std::vector<double>> normals;
    for (int k = 0; k < closest_points.size(); ++k) {
        normals.push_back(estimate_nomal(closest_points[k]));
    }
    t2 = std::chrono::high_resolution_clock::now();
    return normals;
}
