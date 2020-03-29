

#include <kdtree.h>
#include <unordered_set>

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations
	for(int itter{} ; itter < maxIterations ; ++itter){
	
		std::unordered_set<int> tmp_result;

		// Randomly sample subset and fit line
		while(tmp_result.size() < 2)
			tmp_result.insert(rand() % cloud->points.size());

		const pcl::PointXYZ & p1 = cloud->points[*(tmp_result.begin())];
		const pcl::PointXYZ & p2 = cloud->points[*(std::next(tmp_result.begin()))];

		const float a = p1.y - p2.y;
		const float b = p2.x - p1.x;
		const float c = p1.x * p2.y - p2.x * p1.y;

		const float norm_term = std::sqrt(a*a + b*b);

		// Measure distance between every point and fitted line
		for(int idx{} ; idx < cloud->points.size() ; ++ idx){
			// If distance is smaller than threshold count it as inlier

			if(tmp_result.count(idx) > 0)
				continue;

			const auto& tmp_pt = cloud->points[idx];
			if((std::abs(a * tmp_pt.x + b * tmp_pt.y + c)/ norm_term) <= distanceTol )
				tmp_result.insert(idx);

		}

		if(tmp_result.size() > inliersResult.size())
			inliersResult = tmp_result;

	}

	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations
	for(int itter{} ; itter < maxIterations ; ++itter){
	
		std::unordered_set<int> tmp_result;

		// Randomly sample subset and fit line
		while(tmp_result.size() < 3)
			tmp_result.insert(rand() % cloud->points.size());

		const pcl::PointXYZ & p1 = cloud->points[*(tmp_result.begin())];
		const pcl::PointXYZ & p2 = cloud->points[*(std::next(tmp_result.begin()))];
		const pcl::PointXYZ & p3 = cloud->points[*(std::next(std::next(tmp_result.begin())))];

		const float a = (p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y);
		const float b = (p2.z - p1.z) * (p3.x - p1.x) - (p2.x - p1.x) * (p3.z - p1.z);
		const float c = (p2.x - p1.x) * (p2.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
		const float d = - (a*p1.x + b*p1.y + c*p1.z);

		const float norm_term = std::sqrt(a*a + b*b + c*c);

		// Measure distance between every point and fitted line
		for(int idx{} ; idx < cloud->points.size() ; ++ idx){
			// If distance is smaller than threshold count it as inlier

			if(tmp_result.count(idx) > 0)
				continue;

			const auto& tmp_pt = cloud->points[idx];
			if((std::abs(a * tmp_pt.x + b * tmp_pt.y + c*tmp_pt.z + d)/ norm_term) <= distanceTol )
				tmp_result.insert(idx);

		}

		if(tmp_result.size() > inliersResult.size())
			inliersResult = tmp_result;

	}

	return inliersResult;

}
