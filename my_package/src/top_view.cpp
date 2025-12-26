#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

class AroundViewProcessor {
public:
    AroundViewProcessor()
        : canvas_size_(1080, 1090),
          src_points_({{122, 479}, {522, 479}, {269, 219}, {375, 219}}),
          dst_points_({{269, 479}, {375, 479}, {269, 219}, {375, 219}}),
          side_src_points_({{122, 479}, {522, 479}, {269, 219}, {375, 219}}),
          side_dst_points_({{264, 479}, {380, 479}, {264, 219}, {380, 219}})
    {}

    cv::Mat processImages(const cv::Mat& front, const cv::Mat& rear, const cv::Mat& left, const cv::Mat& right, const std::string& vehicle_top_path) {
        // 탑뷰 변환
        cv::Mat front_top_view = transformToTopView(front, src_points_, dst_points_);
        cv::Mat rear_top_view = transformToTopView(rear, src_points_, dst_points_);
        cv::Mat left_top_view = transformToTopView(left, side_src_points_, side_dst_points_);
        cv::Mat right_top_view = transformToTopView(right, side_src_points_, side_dst_points_);

        // 각 이미지 회전
        cv::rotate(rear_top_view, rear_top_view, cv::ROTATE_180);
        cv::rotate(left_top_view, left_top_view, cv::ROTATE_90_COUNTERCLOCKWISE);
        cv::rotate(right_top_view, right_top_view, cv::ROTATE_90_CLOCKWISE);

        // 어라운드 뷰 생성
        cv::Mat around_view = createAroundView(front_top_view, rear_top_view, left_top_view, right_top_view);

        // 차량 윗면 이미지를 추가
        cv::Mat vehicle_top_view = cv::imread(vehicle_top_path, cv::IMREAD_UNCHANGED);
        if (!vehicle_top_view.empty()) {
            attachVehicleTopView(vehicle_top_view, around_view);
        }

        return around_view;
    }

    // 양옆 자르기 함수 추가
    cv::Mat cropSides(const cv::Mat& input_image, int cut_width) {
        int new_width = input_image.cols - 2 * cut_width; // 양옆 잘라낸 너비 계산
        cv::Rect roi(cut_width, 0, new_width, input_image.rows); // 잘라낼 영역 정의
        return input_image(roi); // 자른 이미지 반환
    }

private:
    const cv::Size canvas_size_;
    const std::vector<cv::Point2f> src_points_;
    const std::vector<cv::Point2f> dst_points_;
    const std::vector<cv::Point2f> side_src_points_;
    const std::vector<cv::Point2f> side_dst_points_;

    cv::Mat transformToTopView(const cv::Mat& input_image, const std::vector<cv::Point2f>& src_points, const std::vector<cv::Point2f>& dst_points) {
        cv::Mat homography_matrix = cv::findHomography(src_points, dst_points);
        cv::Mat top_view;
        cv::warpPerspective(input_image, top_view, homography_matrix, cv::Size(640, 480));
        return top_view;
    }

    cv::Mat createAroundView(const cv::Mat& front, const cv::Mat& rear, const cv::Mat& left, const cv::Mat& right) {
        cv::Mat around_view = cv::Mat::zeros(canvas_size_, CV_8UC4);
        std::vector<cv::Mat> views = {front, rear, left, right};
        std::vector<cv::Rect> positions = {
            cv::Rect(225, 0, front.cols, front.rows),
            cv::Rect(225, 610, rear.cols, rear.rows),
            cv::Rect(0, 225, left.cols, left.rows),
            cv::Rect(610, 225, right.cols, right.rows)
        };

        for (size_t i = 0; i < views.size(); i++) {
            cv::Mat view_rgba;
            cv::cvtColor(views[i], view_rgba, cv::COLOR_BGR2BGRA);

            for (int y = 0; y < view_rgba.rows; y++) {
                for (int x = 0; x < view_rgba.cols; x++) {
                    cv::Vec4b& pixel = view_rgba.at<cv::Vec4b>(y, x);
                    if (pixel[0] == 0 && pixel[1] == 0 && pixel[2] == 0) {
                        pixel[3] = 0;
                    }
                }
            }

            for (int y = 0; y < view_rgba.rows; y++) {
                for (int x = 0; x < view_rgba.cols; x++) {
                    cv::Vec4b& pixel = view_rgba.at<cv::Vec4b>(y, x);
                    if (pixel[3] != 0) {
                        around_view.at<cv::Vec4b>(y + positions[i].y, x + positions[i].x) = pixel;
                    }
                }
            }
        }

        return around_view;
    }

    void attachVehicleTopView(const cv::Mat& vehicle_top, cv::Mat& around_view) {
        cv::Mat resized_vehicle_top;
        cv::resize(vehicle_top, resized_vehicle_top, cv::Size(150, 190), 0, 0, cv::INTER_LINEAR);

        int start_x = (around_view.cols - resized_vehicle_top.cols) / 2;
        int start_y = (around_view.rows - resized_vehicle_top.rows) / 2;

        for (int y = 0; y < resized_vehicle_top.rows; y++) {
            for (int x = 0; x < resized_vehicle_top.cols; x++) {
                cv::Vec4b vehicle_pixel = resized_vehicle_top.at<cv::Vec4b>(y, x);
                if (vehicle_pixel[3] != 0) {
                    int canvas_x = start_x + x;
                    int canvas_y = start_y + y;
                    around_view.at<cv::Vec4b>(canvas_y, canvas_x) = vehicle_pixel;
                }
            }
        }
    }
};

class AroundViewNode {
public:
    AroundViewNode(ros::NodeHandle& nh)
        : it_(nh),
          processor_(),
          vehicle_top_path_("/home/a/ERP_42.png") {
        front_sub_ = nh.subscribe("image_jpeg_front/compressed", 1, &AroundViewNode::frontImageCallback, this);
        rear_sub_ = nh.subscribe("image_jpeg_back/compressed", 1, &AroundViewNode::rearImageCallback, this);
        left_sub_ = nh.subscribe("image_jpeg_left/compressed", 1, &AroundViewNode::leftImageCallback, this);
        right_sub_ = nh.subscribe("image_jpeg_right/compressed", 1, &AroundViewNode::rightImageCallback, this);
        around_view_pub_ = it_.advertise("camera/around_view", 1);
    }

    void process() {
        if (!front_img_.empty() && !rear_img_.empty() && !left_img_.empty() && !right_img_.empty()) {
            cv::Mat around_view = processor_.processImages(front_img_, rear_img_, left_img_, right_img_, vehicle_top_path_);
            cv::Mat cropped_view = processor_.cropSides(around_view, 225); // 양옆 자르기
            publishAroundView(cropped_view);
        }
    }

private:
    image_transport::ImageTransport it_;
    image_transport::Publisher around_view_pub_;
    ros::Subscriber front_sub_, rear_sub_, left_sub_, right_sub_;

    cv::Mat front_img_, rear_img_, left_img_, right_img_;
    AroundViewProcessor processor_;
    const std::string vehicle_top_path_;

    void frontImageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
        front_img_ = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    }
    void rearImageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
        rear_img_ = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    }
    void leftImageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
        left_img_ = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    }
    void rightImageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
        right_img_ = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    }

    void publishAroundView(const cv::Mat& around_view) {
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgra8", around_view).toImageMsg();
        around_view_pub_.publish(msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "around_view_node");
    ros::NodeHandle nh;
    AroundViewNode node(nh);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ros::spinOnce();
        node.process();
        loop_rate.sleep();
    }

    return 0;
}