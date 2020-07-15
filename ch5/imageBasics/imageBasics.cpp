#include <iostream>
#include <chrono>

using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


int main(int argc, char *argv[])
{
    // 读取 argv[1]指定的图像
    cv::Mat image;
    image = cv::imread(argv[1]);    // cv::imread函数读取指定路径下的图像

    // 判断图像文件是否正确读取
    if (image.data == nullptr)  //数据不存在，可能是文件不存在
    {
        cerr << "file " << argv[1] << " not excited." << endl;
        return 0;
    }
    
    // 文件顺利读取，首先输出一些基本信息
    cout << "width of image is: " << image.cols << ", height is: " << image.rows
        << " number of channel is: " << image.channels() << endl;

    cv::imshow("image", image);     // 用 cv::imshow 显示图像 
    cv::waitKey(0);     // 暂停程序，等待一个按键输入

    // 判断 image 的类型
    if (image.type() != CV_8UC1 && image.type() != CV_8UC3)
    {
        // 图像类型不符合要求
        cout << "错误！请输入一张彩色图或灰度图." << endl;
        return 0;
    }
    
    // 遍历图像，请注意一下遍历方式也可使用于随机像素访问
    // 使用 std::chrono 给算法计时
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    for (size_t y = 0; y < image.rows; y++)
    {
        // 用 cv::Mat::ptr 获得图像行指针
        unsigned char *row_ptr = image.ptr<unsigned char>(y);   // row_ptr是第y行的头指针
        for (size_t x = 0; x < image.cols; x++)
        {
            // 访问位于x,y处的像素
            unsigned char *data_ptr = &row_ptr[x * image.channels()];   // data_ptr 指向待访问的像素数据
            // 输出该像素的每个通道，如果是灰度图就只有一个通道
            for (int c = 0; c != image.channels(); c++)
            {
                unsigned char data = data_ptr[c];   // data 为 I(x,y) 第c个通道的值
            }
        } 
    }
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast < chrono::duration < double >> (t2 - t1);
    cout << "time of ergodic image: " << time_used.count() << "s" << endl;

    // 关于 cv::Mat 的拷贝
    // 直接复制并不会拷贝数据
    cv::Mat image_another = image;
    // 修改 image_another 会导致 image 发生变化
    image_another(cv::Rect(0, 0, 100, 100)).setTo(0);       // 将左上交100*100的块置零
    cv::imshow("image", image);
    cv::waitKey(0);

    // 使用clone函数来拷贝数据
    cv::Mat image_clone = image.clone();
    image_clone(cv::Rect(0, 0, 100, 100)).setTo(255);
    cv::imshow("image", image);
    cv::imshow("image clone", image_clone);
    cv::waitKey(0);

    // 对于图像还有很多基本的操作，如剪切，旋转，缩放等，请参看OpenCV官方文档查询
    cv::destroyAllWindows();
    return 0;

}