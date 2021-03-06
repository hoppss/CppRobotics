#include <opencv2/opencv.hpp>
#include <iostream>


using namespace std;
using namespace cv;


int main(int argc, char** argv){
    vector<cv::Point> points;

    points.push_back(Point(27, 39));
    points.push_back(Point(8, 5));
    points.push_back(Point(8, 9));
    points.push_back(Point(16, 22));
    points.push_back(Point(44, 71));
    points.push_back(Point(35, 44));
    points.push_back(Point(43, 57));
    points.push_back(Point(19, 24));
    points.push_back(Point(27, 39));
    points.push_back(Point(37, 52));

    Mat src = Mat::zeros(400, 400, CV_8UC3);

    for (int i = 0;i < points.size();i++){
        //在原图上画出点
        circle(src, points[i], 3, Scalar(0, 0, 255), 1, 8);
    }
    //构建A矩阵
    int N = 2;
    Mat A = Mat::zeros(N, N, CV_64FC1);

    for (int row = 0;row < A.rows;row++)
    {
        for (int col = 0; col < A.cols;col++)
        {
            for (int k = 0;k < points.size();k++)
            {
                A.at<double>(row, col) = A.at<double>(row, col) + pow(points[k].x, row + col);
            }
        }
    }
    //构建B矩阵
    Mat B = Mat::zeros(N, 1, CV_64FC1);
    for (int row = 0;row < B.rows;row++)
    {
        for (int k = 0;k < points.size();k++)
        {
            B.at<double>(row, 0) = B.at<double>(row, 0) + pow(points[k].x, row)*points[k].y;
        }
    }
    //A*X=B
    Mat X;
    //cout << A << endl << B << endl;
    cv::solve(A, B, X,DECOMP_LU);
    cout << "X:(b,a)"<< X << endl;
    vector<Point>lines;
    for (int x = 0;x < src.size().width;x++)
    {				// y = b + ax;
        double y = X.at<double>(0, 0) + X.at<double>(1, 0)*x;
        printf("(%d,%lf)\n", x, y);
        lines.push_back(Point(x, y));
    }
    polylines(src, lines, false, Scalar(255, 0, 0), 1, 8);
    imshow("src", src);

    //imshow("src", A);
    waitKey(0);

    cout << "fitLine function " << endl;

    cv::Vec4f line_para;
    cv::fitLine(points, line_para, cv::DIST_L2, 0, 1e-2, 1e-2);

    std::cout << "line_para = " << line_para << std::endl;

    //获取点斜式的点和斜率
    cv::Point point0;
    point0.x = line_para[2];
    point0.y = line_para[3];

    double k = line_para[1] / line_para[0];

    //计算直线的端点(y = k(x - x0) + y0)
    cv::Point point1, point2;
    point1.x = 0;
    point1.y = k * (0 - point0.x) + point0.y;
    point2.x = 400;
    point2.y = k * (400 - point0.x) + point0.y;

    cv::line(src, point1, point2, cv::Scalar(0, 255, 0), 2, 8, 0);
    waitKey(0);


    return 0;
}
