#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv4/opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Vector3.h>

class PenampilDeteksi
{

private:
    cv::Mat citra_kamera_;

    cv::Point2f koordinat_bola_kamera_;
    cv::Point2f koordinat_bola_kamera_kartesian_;
    int radius_bola_;

    bool mulai_ = true;

    bool sudah_siap_ = true;

    void cari_warna_bola() {

        cv::Rect2d potongan = cv::selectROI(citra_kamera_);

        cv::Mat potongan_citra = citra_kamera_(potongan);
        cv::cvtColor(potongan_citra,potongan_citra, cv::COLOR_BGR2HSV);

    }

public:
    void set_citra_kamera(cv::Mat *citra_kamera)
    {
        citra_kamera_ = *citra_kamera;

        if (citra_kamera_.cols > 0 && citra_kamera_.rows > 0) {
            sudah_siap_ = true;
        } else {
            sudah_siap_ = false;
        }

    }

    void set_koordinat_bola_kamera(int x, int y)
    {
        koordinat_bola_kamera_.x = x;
        koordinat_bola_kamera_.y = y;
    }

    void set_radius_bola(int radius)
    {
        radius_bola_ = radius;
    }

    void set_koordinat_bola_kamera_kartesian(int x, int y)
    {
        koordinat_bola_kamera_kartesian_.x = x;
        koordinat_bola_kamera_kartesian_.y = y;
    }

    bool get_mulai() {
        return mulai_;
    }

    void main_penampil_deteksi()
    {

        if (sudah_siap_) {

            cv::circle(
                citra_kamera_,
                koordinat_bola_kamera_,
                radius_bola_,
                cv::Scalar(0, 255, 0),
                2);

            cv::putText(
                citra_kamera_,
                "(" + std::to_string((int)(koordinat_bola_kamera_kartesian_.x)) + "px," + std::to_string((int)(koordinat_bola_kamera_kartesian_.y)) + "px)",
                cv::Point(
                    (koordinat_bola_kamera_.x + 10),
                    (koordinat_bola_kamera_.y + 10)),
                cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0, 0, 255), 2, 8);

            cv::imshow("Citra Kamera", citra_kamera_);

            char c = cv::waitKey(1);

            if (c == 'q')
            {
                
                mulai_ = false;

            } else if (c == 'b') {



            }
        }
    }
};

class PenampilDeteksiNode
{
private:

    PenampilDeteksi penampil_deteksi_;

    ros::NodeHandle nh_;
    ros::Rate loop_rate_;
    image_transport::ImageTransport it_;

    image_transport::Subscriber citra_kamera_sub_;

    cv_bridge::CvImagePtr cv_ptr_;
    cv::Mat citra_kamera_;

    geometry_msgs::Vector3 koordinat_bola_kamera_;
    ros::Subscriber koordinat_bola_kamera_sub_;

    geometry_msgs::Vector3 koordinat_bola_kamera_kartesian_;
    ros::Subscriber koordinat_bola_kamera_kartesian_sub_;

    bool siap_tampil_ = false;

    void koordinat_bola_kamera_cb(const geometry_msgs::Vector3ConstPtr& data) {
        koordinat_bola_kamera_ = *data;
    }

    void koordinat_bola_kamera_kartesian_cb(const geometry_msgs::Vector3ConstPtr& data) {
        koordinat_bola_kamera_kartesian_ = *data;
    }

    void citra_kamera_cb(const sensor_msgs::ImageConstPtr &data)
    {
        try
        {
            cv_ptr_ = cv_bridge::toCvCopy(
                data,
                sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        if (cv_ptr_->image.rows > 0 && cv_ptr_->image.cols > 0)
        {
            citra_kamera_ = cv_ptr_->image;
            siap_tampil_ = true;
        }
        else
        {
            siap_tampil_ = false;
        }
    }

public:
    PenampilDeteksiNode() : it_(nh_), loop_rate_(15) {

        citra_kamera_sub_ = it_.subscribe(
            "kamera/image_raw",
            1,
            &PenampilDeteksiNode::citra_kamera_cb,
            this);

        koordinat_bola_kamera_sub_ = 
            nh_.subscribe(
                "koordinat_bola_kamera",
                1,
                &PenampilDeteksiNode::koordinat_bola_kamera_cb,
                this
            );
        
        koordinat_bola_kamera_kartesian_sub_ =
            nh_.subscribe(
                "koordinat_bola_kamera_kartesian",
                1,
                &PenampilDeteksiNode::koordinat_bola_kamera_kartesian_cb,
                this
            );

    }

    void main_penampil_deteksi() {

        while (ros::ok() && penampil_deteksi_.get_mulai()) {

            if (siap_tampil_) {

                penampil_deteksi_.set_citra_kamera(&citra_kamera_);
                penampil_deteksi_.set_koordinat_bola_kamera(
                    koordinat_bola_kamera_.x,
                    koordinat_bola_kamera_.y
                );
                penampil_deteksi_.set_koordinat_bola_kamera_kartesian(
                    koordinat_bola_kamera_kartesian_.x,
                    koordinat_bola_kamera_kartesian_.y
                );
                penampil_deteksi_.set_radius_bola(koordinat_bola_kamera_.z);

                penampil_deteksi_.main_penampil_deteksi();

            }

            ros::spinOnce();
            loop_rate_.sleep();
        }

    }

    
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "penampil_deteksi");

    PenampilDeteksiNode node;

    node.main_penampil_deteksi();

    return 0;

}
