// PCL lib Functions for processing point clouds

#include "processPointClouds.h"

template <typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // TODO: Fill in this function

    // For max iterations
    for (int itter{}; itter < maxIterations; ++itter)
    {

        std::unordered_set<int> tmp_result;

        // Randomly sample subset and fit line
        while (tmp_result.size() < 3)
            tmp_result.insert(rand() % cloud->points.size());

        const PointT &p1 = cloud->points[*(tmp_result.begin())];
        const PointT &p2 = cloud->points[*(std::next(tmp_result.begin()))];
        const PointT &p3 = cloud->points[*(std::next(std::next(tmp_result.begin())))];

        const float a = (p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y);
        const float b = (p2.z - p1.z) * (p3.x - p1.x) - (p2.x - p1.x) * (p3.z - p1.z);
        const float c = (p2.x - p1.x) * (p2.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
        const float d = -(a * p1.x + b * p1.y + c * p1.z);

        const float norm_term = std::sqrt(a * a + b * b + c * c);

        // Measure distance between every point and fitted line
        for (int idx{}; idx < cloud->points.size(); ++idx)
        {
            // If distance is smaller than threshold count it as inlier

            if (tmp_result.count(idx) > 0)
                continue;

            const auto &tmp_pt = cloud->points[idx];
            if ((std::abs(a * tmp_pt.x + b * tmp_pt.y + c * tmp_pt.z + d) / norm_term) <= distanceTol)
                tmp_result.insert(idx);
        }

        if (tmp_result.size() > inliersResult.size())
            inliersResult = tmp_result;
    }

    return inliersResult;
}

//constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

//de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    typename pcl::PointCloud<PointT>::Ptr filtered{new pcl::PointCloud<PointT>};
    typename pcl::PointCloud<PointT>::Ptr cropped{new pcl::PointCloud<PointT>};
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*filtered);

    pcl::CropBox<PointT> roi{true};
    roi.setMin(minPoint);
    roi.setMax(maxPoint);
    roi.setInputCloud(filtered);

    roi.filter(*cropped);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cropped);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    for (int point : indices)
    {
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cropped);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cropped);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cropped;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    pcl::ExtractIndices<PointT> extract;

    typename pcl::PointCloud<PointT>::Ptr obst{new pcl::PointCloud<PointT>};
    typename pcl::PointCloud<PointT>::Ptr road{new pcl::PointCloud<PointT>};

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);

    extract.setNegative(true);
    extract.filter(*obst);

    extract.setNegative(false);
    extract.filter(*road);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obst, road);

    return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients{new pcl::ModelCoefficients};

    // TODO:: Fill in this function to find inliers for the cloud.
#if defined PCL_IMP
    pcl::SACSegmentation<PointT> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
#else

    auto inliners_set = RansacPlane(cloud, maxIterations, distanceThreshold);
    inliers->indices.assign(inliners_set.begin(), inliners_set.end());
#endif
    if (inliers->indices.size() == 0)
    {
        std::cout << "No plane found in given point cloud ." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

#if defined PCL_IMP
    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    pcl::EuclideanClusterExtraction<PointT> ec;
    typename pcl::search::KdTree<PointT>::Ptr tree{new pcl::search::KdTree<PointT>};
    std::vector<pcl::PointIndices> clusterIndices;

    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);

    ec.extract(clusterIndices);


    for (const auto &current_cluster_indices : clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr tmp_cluster{new pcl::PointCloud<PointT>};
        tmp_cluster->width = clusterIndices.size();
        tmp_cluster->is_dense = true;
        tmp_cluster->height = 1;

        for (const auto idx : current_cluster_indices.indices)
            tmp_cluster->points.push_back(cloud->points[idx]);

        clusters.push_back(tmp_cluster);
    }
#else
    KdTree kdtree_obj{};
    std::vector<std::vector<float>> points{};

    for (int idx{} ; idx < cloud->points.size(); ++idx){
        const auto &pt = cloud->points[idx];
        kdtree_obj.insert({pt.x, pt.y, pt.z}, idx);
        points.emplace_back(std::vector<float>({pt.x, pt.y, pt.z}));
    }

    auto custom_clusters = euclideanCluster(points, &kdtree_obj,clusterTolerance);

    for(const auto& idx_vec : custom_clusters){

        typename pcl::PointCloud<PointT>::Ptr tmp_cluster{new pcl::PointCloud<PointT>};
        tmp_cluster->width = idx_vec.size();
        tmp_cluster->is_dense = true;
        tmp_cluster->height = 1;

        for(const auto & idx : idx_vec)
            tmp_cluster->points.push_back(cloud->points[idx]);

        clusters.push_back(tmp_cluster);
    }
#endif
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}