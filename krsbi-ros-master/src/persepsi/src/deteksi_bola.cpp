#include <ros/ros.h>
#include <opencv4/opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Vector3.h>

double map_num(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

class DeteksiObjek
{

private:
    int panjang_citra_ = 0;
    int tinggi_citra_ = 0;

    cv::Point2f titik_tengah_citra_;

    cv::Mat citra_kamera_;
    cv::Mat citra_kamera_hsv_;

    cv::Scalar warna_objek_terang_;
    cv::Scalar warna_objek_gelap_;

    cv::Mat mask_filter_warna_objek_;
    cv::Mat mask_flood_fill_;

    float radius_objek_;
    cv::Point2f koordinat_objek_kamera_;
    cv::Point2f koordinat_objek_kamera_kartesian_;

    std::vector<std::vector<cv::Point>> contours_;
    std::vector<cv::Vec4i> hierarchy_;

    void filter_warna_objek()
    {

        cv::cvtColor(citra_kamera_, citra_kamera_hsv_, cv::COLOR_BGR2HSV);

        cv::inRange(
            citra_kamera_hsv_,
            warna_objek_gelap_,
            warna_objek_terang_,
            mask_filter_warna_objek_);

        cv::morphologyEx(
            mask_filter_warna_objek_,
            mask_filter_warna_objek_,
            cv::MORPH_OPEN,
            cv::Mat::ones(cv::Size(3, 3),
                          CV_8UC1));

        cv::morphologyEx(
            mask_filter_warna_objek_,
            mask_filter_warna_objek_,
            cv::MORPH_CLOSE,
            cv::Mat::ones(cv::Size(8, 8),
                          CV_8UC1));

        cv::findContours(
            mask_filter_warna_objek_,
            contours_, hierarchy_,
            cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE,
            cv::Point(0, 0));

        if (contours_.size() > 0)
        {

            std::vector<std::vector<cv::Point>> hull(contours_.size());

            for (int i = 0; i < contours_.size(); i++)
            {

                cv::convexHull(
                    cv::Mat(contours_[i]),
                    hull[i],
                    false);
            }

            cv::fillPoly(
                mask_filter_warna_objek_, hull,
                cv::Scalar(255), cv::LINE_8, 0,
                cv::Point());

            mask_flood_fill_ = mask_filter_warna_objek_.clone();
            cv::floodFill(mask_flood_fill_, cv::Point(0, 0), cv::Scalar(255));
            cv::bitwise_not(mask_flood_fill_, mask_flood_fill_);

            mask_filter_warna_objek_ = (mask_filter_warna_objek_ | mask_flood_fill_);
        }
    }

    void cari_koordinat_objek_kamera()
    {

        contours_.clear();
        hierarchy_.clear();

        cv::findContours(
            mask_filter_warna_objek_, contours_,
            hierarchy_, cv::RETR_TREE,
            cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

        if (contours_.size() > 0)
        {
            int max_area = 0;

            for (int i = 0; i < contours_.size(); i++)
            {

                double area_0 = cv::contourArea(contours_[max_area]);
                double area = cv::contourArea(contours_[i]);
                if (area > area_0)
                {
                    max_area = i;
                }
            }

            cv::minEnclosingCircle(
                contours_[max_area],
                koordinat_objek_kamera_,
                radius_objek_);
        }
    }

    void cari_koordinat_objek_kamera_kartesian()
    {
        koordinat_objek_kamera_kartesian_.x = map_num(
            koordinat_objek_kamera_.x,
            0.0,
            panjang_citra_,
            (-1.0 * titik_tengah_citra_.x),
            titik_tengah_citra_.x);

        koordinat_objek_kamera_kartesian_.y = map_num(
            koordinat_objek_kamera_.y,
            0.0,
            tinggi_citra_,
            titik_tengah_citra_.y,
            (-1.0 * titik_tengah_citra_.y));
    }

public:
    void set_citra_kamera(cv::Mat *citra_kamera)
    {
        this->citra_kamera_ = *citra_kamera;

        panjang_citra_ = this->citra_kamera_.cols;
        tinggi_citra_ = this->citra_kamera_.rows;

        titik_tengah_citra_.x = panjang_citra_ / 2.0;
        titik_tengah_citra_.y = panjang_citra_ / 2.0;
    }

    void set_warna_objek_terang(
        int hue,
        int sat,
        int val)
    {

        warna_objek_terang_ = cv::Scalar(
            hue,
            sat,
            val);
    }

    void set_warna_objek_gelap(
        int hue,
        int sat,
        int val)
    {
        warna_objek_gelap_ = cv::Scalar(
            hue,
            sat,
            val);
    }

    cv::Mat *get_mask_filter_warna_objek()
    {
        return &mask_filter_warna_objek_;
    }

    cv::Point2f *get_koordinat_objek_kamera()
    {
        return &koordinat_objek_kamera_;
    }

    cv::Point2f *get_koordinat_objek_kamera_kartesian()
    {
        return &koordinat_objek_kamera_kartesian_;
    }

    float get_radius_objek()
    {
        return radius_objek_;
    }

    void main_deteksi_objek()
    {
        filter_warna_objek();
        cari_koordinat_objek_kamera();
        cari_koordinat_objek_kamera_kartesian();
    }
};

class DeteksiBolaNode
{

private:
    bool siap_deteksi_ = false;

    ros::NodeHandle nh_;
    ros::Rate loop_rate_;
    image_transport::ImageTransport it_;

    image_transport::Subscriber citra_kamera_sub_;

    cv_bridge::CvImagePtr cv_ptr_;
    cv::Mat citra_kamera_;

    geometry_msgs::Vector3 koordinat_bola_kamera_;
    ros::Publisher koordinat_bola_kamera_pub_;

    geometry_msgs::Vector3 koordinat_bola_kamera_kartesian_;
    ros::Publisher koordinat_bola_kamera_kartesian_pub_;

    ros::Subscriber warna_bola_gelap_sub_;
    ros::Subscriber warna_bola_terang_sub_;

    DeteksiObjek deteksi_bola;

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
            siap_deteksi_ = true;
        }
        else
        {
            siap_deteksi_ = false;
        }
    }

    void warna_bola_gelap_cb(const geometry_msgs::Vector3ConstPtr& data) {
        deteksi_bola.set_warna_objek_gelap(
            data->x,
            data->y,
            data->z
        );
    }

    void warna_bola_terang_cb(const geometry_msgs::Vector3ConstPtr& data) {
        deteksi_bola.set_warna_objek_terang(
            data->x,
            data->y,
            data->z
        );
    }


public:
    DeteksiBolaNode() : it_(nh_), loop_rate_(15)
    {
        citra_kamera_sub_ = it_.subscribe(
            "kamera/image_raw",
            1,
            &DeteksiBolaNode::citra_kamera_cb,
            this);
        warna_bola_gelap_sub_ = nh_.subscribe(
            "warna_bola_gelap", 
            1, 
            &DeteksiBolaNode::warna_bola_gelap_cb, 
            this
        );
        warna_bola_terang_sub_ = nh_.subscribe(
            "warna_bola_terang", 
            1, 
            &DeteksiBolaNode::warna_bola_terang_cb, 
            this
        );

        koordinat_bola_kamera_pub_ = 
            nh_.advertise<geometry_msgs::Vector3>("koordinat_bola_kamera", 1);
        koordinat_bola_kamera_kartesian_pub_ =
            nh_.advertise<geometry_msgs::Vector3>("koordinat_bola_kamera_kartesian", 1);

        deteksi_bola.set_warna_objek_gelap(0, 100, 20);
        deteksi_bola.set_warna_objek_terang(10, 255, 255);
    }

    void main_deteksi_objek()
    {
        while (ros::ok())
        {

            if (siap_deteksi_)
            {

                deteksi_bola.set_citra_kamera(&citra_kamera_);
                deteksi_bola.main_deteksi_objek();

                koordinat_bola_kamera_.x = deteksi_bola.get_koordinat_objek_kamera()->x;
                koordinat_bola_kamera_.y = deteksi_bola.get_koordinat_objek_kamera()->y;
                koordinat_bola_kamera_.z = deteksi_bola.get_radius_objek();

                koordinat_bola_kamera_kartesian_.x = deteksi_bola.get_koordinat_objek_kamera_kartesian()->x;
                koordinat_bola_kamera_kartesian_.y = deteksi_bola.get_koordinat_objek_kamera_kartesian()->y;
                koordinat_bola_kamera_kartesian_.z = deteksi_bola.get_radius_objek();
                
                koordinat_bola_kamera_pub_.publish(koordinat_bola_kamera_);
                koordinat_bola_kamera_kartesian_pub_.publish(koordinat_bola_kamera_kartesian_);
                
            }
            loop_rate_.sleep();
            ros::spinOnce();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "deteksi_objek");

    DeteksiBolaNode node;

    node.main_deteksi_objek();

    return 0;
}