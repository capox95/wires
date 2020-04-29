#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/intersections.h>

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_i(new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *source) == -1)
    {
        PCL_ERROR(" error opening file ");
        return (-1);
    }

    // -- GROUND PLANE -------------------------------------------------------------
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.02);

    seg.setInputCloud(source);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;

    // --------------------------------------------------------------------------------

    // SOURCE_I
    pcl::copyPointCloud(*source, *source_i);
    for (int i = 0; i < inliers->indices.size(); i++)
    {
        source_i->points[inliers->indices[i]].z = -coefficients->values[3];
        source_i->points[inliers->indices[i]].intensity = 1;
    }

    // FILL NANs ---------------------------------------------------------------------------
    pcl::PointXYZI pointC, newCoordinates;

    int counterN = 0;
    for (int i = 2; i < source_i->width - 2; i++)
    {
        for (int j = 2; j < source_i->height - 2; j++)
        {

            counterN = 0;
            newCoordinates.x = 0;
            newCoordinates.y = 0;
            newCoordinates.z = 0;
            newCoordinates.intensity = 0;

            pointC = source_i->at(i, j);
            if ((std::isnan(pointC.x)) || (std::isnan(pointC.y)) || (std::isnan(pointC.z)))
            {
                for (int ii = i - 2; ii < i + 2; ii++)
                {
                    for (int jj = j - 2; jj < j + 2; jj++)
                    {
                        if (!(std::isnan(source_i->at(ii, jj).x)) && (!std::isnan(source_i->at(ii, jj).y)) && (!std::isnan(source_i->at(ii, jj).z)))
                        {
                            if (source_i->at(ii, jj).intensity == 1)
                            {
                                newCoordinates.x += source_i->at(ii, jj).x;
                                newCoordinates.y += source_i->at(ii, jj).y;
                                newCoordinates.z += source_i->at(ii, jj).z;
                                counterN++;
                            }
                        }
                    }
                }
            }
            if (counterN != 0)
            {
                newCoordinates.x /= counterN;
                newCoordinates.y /= counterN;
                newCoordinates.z /= counterN;
                newCoordinates.intensity = 1;

                source_i->at(i, j) = newCoordinates;
            }
        }
    }

    /*
        int counter = 0;
        for (int i = 0; i < source_i->size(); i++)
        {
            if ((!std::isnan(source_i->points[i].x)) && (!std::isnan(source_i->points[i].y)) && (!std::isnan(source_i->points[i].z)))
                counter++;
        }
        std::cout << "counter result: " << counter << std::endl;
        std::cout << "source size: " << source->size() << std::endl;
        */

    Eigen::Vector3f x0;
    Eigen::Vector3f n(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    float p = coefficients->values[3];
    float distance;

    for (int i = 0; i < source_i->size(); i++)
    {
        if ((!std::isnan(source_i->points[i].x)) && (!std::isnan(source_i->points[i].y)) && (!std::isnan(source_i->points[i].z)))
        {

            x0 = source_i->points[i].getVector3fMap();
            distance = n.dot(x0) + p;

            if (distance > 0)
                source_i->points[i].z = -p;
        }
        else
        {
            source_i->points[i].z = -p;
            source_i->points[i].x = 0;
            source_i->points[i].y = 0;
        }
    }

    // -- GROUND PLANE UPDATE-------------------------------------------------------------
    pcl::ModelCoefficients::Ptr coefficientsUp(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliersUp(new pcl::PointIndices());
    pcl::SACSegmentation<pcl::PointXYZI> seg2;
    seg2.setOptimizeCoefficients(true);
    seg2.setModelType(pcl::SACMODEL_PLANE);
    seg2.setMethodType(pcl::SAC_RANSAC);
    seg2.setMaxIterations(1000);
    seg2.setDistanceThreshold(0.01);

    seg2.setInputCloud(source_i);
    seg2.segment(*inliersUp, *coefficientsUp);

    if (inliersUp->indices.size() == 0)
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;

    for (int k = 0; k < inliersUp->indices.size(); k++)
        source_i->points[inliersUp->indices[k]].intensity = 1;

    for (int i = 0; i < source_i->size(); i++)
    {
        if (source_i->points[i].intensity != 1)
            source_i->points[i].z = 0;
    }
    /*
    std::vector<pcl::ModelCoefficients> lines;
    bool flag_lines = false;
    while (lines.size() < 2)
    {
        std::vector<int> indices_cloud;
        for (int i = 0; i < source_i->size(); i++)
        {
            if (source_i->points[i].intensity == 0)
                indices_cloud.push_back(i);
        }

        // -- LINE FITTING -------------------------------------------------------------

        pcl::ModelCoefficients line;
        pcl::PointIndices::Ptr inliers_line(new pcl::PointIndices());
        pcl::SACSegmentation<pcl::PointXYZI> segline;
        segline.setOptimizeCoefficients(true);
        segline.setModelType(pcl::SACMODEL_LINE);
        segline.setMethodType(pcl::SAC_RANSAC);
        segline.setMaxIterations(1000);
        segline.setDistanceThreshold(0.005);

        pcl::PointIndicesPtr seg_indices(new pcl::PointIndices);
        seg_indices->indices = indices_cloud;
        segline.setIndices(seg_indices);

        segline.setInputCloud(source_i);
        segline.segment(*inliers_line, line);

        if (inliers->indices.size() == 0)
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;

        // line with line intersaction da provare
        for (int i = 0; i < inliers_line->indices.size(); i++)
        {
            if (lines.size() < 1)
                source_i->points[inliers_line->indices[i]].intensity = 0.5;
            else if (lines.size() < 3)
                source_i->points[inliers_line->indices[i]].intensity = 0.7;
            else
                source_i->points[inliers_line->indices[i]].intensity = 0.5;
        }
        lines.push_back(line);
    }
    */

    //pcl::copyPointCloud(*source_i, *source);
    //pcl::io::savePCDFile("new.pcd", *source);

    pcl::visualization::PCLVisualizer vizProcessed("PCL Processed Cloud");
    vizProcessed.setBackgroundColor(1.0f, 1.0f, 1.0f);
    //vizSource.addPointCloud<pcl::PointXYZ>(source, "source");
    //vizSource.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 0.0f, "source");
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(source_i, "intensity");
    vizProcessed.addPointCloud<pcl::PointXYZI>(source_i, intensity_distribution, "source_i");
    vizProcessed.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "source_i");

    pcl::visualization::PCLVisualizer vizSource("PCL Source Cloud");
    vizSource.addCoordinateSystem(0.05, "coordinate");
    vizSource.setBackgroundColor(1.0f, 1.0f, 1.0f);
    vizSource.addPointCloud<pcl::PointXYZ>(source, "source");
    vizSource.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 0.0f, "source");

    //for (int i = 0; i < lines.size(); i++)
    //    vizSource.addLine(lines[i], "line" + std::to_string(i));
    //vizSource.addPointCloud<pcl::PointXYZI>(source_wires, "source_wires");
    //vizSource.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5f, 0.0f, 1.0f, "source_wires");

    while (!vizSource.wasStopped())
    {
        vizSource.spinOnce();
    }

    return 0;
}
