#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc/slic.hpp>

using namespace cv;

/** Compute the mean coordinates of each superpixel (i.e. spatial centroids).
 * [in] labels a matrix of type CV_32SC1 holding the labels for each pixel
 * [out] means the spatial centroids (or means in y and x axes) of the superpixels
 */
void getMeans(const cv::Mat &labels, std::vector<cv::Vec2f> &means)
{

    // Count superpixels or get highest superpixel index:
    int superpixels = 0;
    //ge number of labels
    double min, max;
    cv::minMaxLoc(labels, &min, &max);
    std::cout << "num labels: " << superpixels << std::endl;

    superpixels = max + 1;

    // Setup means as zero vectors.
    means.clear();
    means.resize(superpixels);
    for (int k = 0; k < superpixels; k++)
    {
        means[k] = cv::Vec2f(0, 0);
    }

    std::vector<int> counts(superpixels, 0);

    // Sum y and x coordinates for each superpixel:
    for (int i = 0; i < labels.rows; ++i)
    {
        for (int j = 0; j < labels.cols; ++j)
        {
            means[labels.at<int>(i, j)][0] += i; // for computing mean i (i.e. row or y axis)
            means[labels.at<int>(i, j)][1] += j; // for computing the mean j (i.e. column or x axis)

            counts[labels.at<int>(i, j)]++;
        }
    }

    // Obtain averages by dividing by the size (=number of pixels) of the superpixels.
    for (int k = 0; k < superpixels; ++k)
    {
        means[k] /= counts[k];
    }
}

Mat segmentImage(Mat image)
{
    int num_iterations = 4;
    int prior = 2;
    bool double_step = false;
    int num_levels = 10;
    int num_histogram_bins = 5;

    int width, height;

    width = image.size().width;
    height = image.size().height;

    Ptr<cv::ximgproc::SuperpixelSLIC> seeds = cv::ximgproc::createSuperpixelSLIC(image, cv::ximgproc::SLIC, 20, 10.0f);

    Mat mask;

    seeds->iterate(num_iterations);

    Mat labels;
    seeds->getLabels(labels);

    std::cout << labels.size() << std::endl;

    /*
    for (int i = 0; i < labels.rows; i++)
    {
        for (int j = 0; j < labels.cols; j++)
        {
            if (labels.at<int>(i, j) == 0)
            {
                //std::cout << i << " " << j << " " << labels.at<int>(i, j) << std::endl;
                //image.at<uchar>(i, j) = 127;
            }
        }
    }
    */

    std::ofstream myfile;
    myfile.open("label.txt");
    myfile << labels;
    myfile.close();

    Mat image_mask = image;
    cvtColor(image_mask, image_mask, 8);
    seeds->getLabelContourMask(mask, false);

    std::vector<cv::Vec2f> means;
    getMeans(labels, means);

    for (int i = 0; i < means.size(); i++)
    {
        int row = round(means[i][0]);
        int col = round(means[i][1]);
        mask.at<uchar>(row, col) = 255;
    }

    image_mask.setTo(Scalar(0, 0, 255), mask);

    namedWindow("result", WINDOW_AUTOSIZE);
    imshow("result", image_mask);
    imwrite("result.png", image_mask);
    return labels;
}

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        printf("usage: DisplayImage.out <Image_Path>\n");
        return -1;
    }
    Mat image;
    image = imread(argv[1], 0);
    if (!image.data)
    {
        printf("No image data \n");
        return -1;
    }
    namedWindow("Display Image", WINDOW_AUTOSIZE);
    imshow("Display Image", image);

    // compute binary image to be 100% sure
    Mat binary;
    threshold(image, binary, 127, 255, THRESH_BINARY);

    // compute superpixels
    Mat labels = segmentImage(binary);

    //namedWindow("Display Image Binary", WINDOW_AUTOSIZE);
    //imshow("Display Image Binary", binary);

    waitKey(0);
    return 0;
}
