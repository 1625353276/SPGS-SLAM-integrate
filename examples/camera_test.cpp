/**
 * Camera Test - 实时显示摄像头画面（WSLg GUI窗口）
 */

#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>

using namespace std;

int main(int argc, char** argv)
{
    int cam_id = (argc > 1) ? atoi(argv[1]) : 0;
    int width  = (argc > 2) ? atoi(argv[2]) : 640;
    int height = (argc > 3) ? atoi(argv[3]) : 480;

    cout << "Opening camera " << cam_id << " at " << width << "x" << height << endl;

    cv::VideoCapture cap(cam_id, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        cerr << "Failed to open camera " << cam_id << endl;
        return 1;
    }

    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cap.set(cv::CAP_PROP_FPS, 30);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

    cout << "Camera opened: "
         << (int)cap.get(cv::CAP_PROP_FRAME_WIDTH) << "x"
         << (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT) << " @ "
         << cap.get(cv::CAP_PROP_FPS) << "fps" << endl;
    cout << "Press 'q' or ESC to quit" << endl;

    cv::namedWindow("Camera", cv::WINDOW_AUTOSIZE);

    cv::Mat frame;
    int frame_count = 0;
    int empty_count = 0;
    auto t_start = chrono::steady_clock::now();

    while (true) {
        cap >> frame;

        if (frame.empty()) {
            empty_count++;
            if (empty_count % 30 == 0)
                cerr << "[WARN] Empty frames: " << empty_count << endl;
            continue;
        }

        frame_count++;

        // 计算 FPS 并叠加到画面上
        auto t_now = chrono::steady_clock::now();
        double elapsed = chrono::duration_cast<chrono::milliseconds>(t_now - t_start).count() / 1000.0;
        double fps = (elapsed > 0) ? frame_count / elapsed : 0;

        string info = "FPS: " + to_string((int)fps) + "  Frame: " + to_string(frame_count);
        cv::putText(frame, info, cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

        cv::imshow("Camera", frame);

        // 按 q 或 ESC 退出
        int key = cv::waitKey(1);
        if (key == 'q' || key == 27) break;
    }

    cout << "Total frames: " << frame_count << ", Empty: " << empty_count << endl;
    cap.release();
    cv::destroyAllWindows();
    return 0;
}
