#include "geometry_msgs/msg/point.hpp"
#include "math.h"
#include "miradar.h"
#include "miradar_msgs/msg/ppi.hpp"
#include "miradar_msgs/msg/ppi_data.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"

#define DEG2RAG(deg) (((deg) / 360) * 2 * M_PI)

class MiRadarROS2 {
public:
    rcl_interfaces::msg::SetParametersResult setParam(
        const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        RCLCPP_INFO(node->get_logger(), "Applying Parameter Change.");
        for (const auto &parameter : parameters) {
            setParamImpl(parameter);
        }

        isConfigUpdate = true;
        return result;
    }

    void setParamImpl(const rclcpp::Parameter &parameter) {
        if (parameter.get_name() == "devicename" &&
            parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
            deviceFile = parameter.as_string();
        } else if (parameter.get_name() == "sensor_mode" &&
                   parameter.get_type() ==
                       rclcpp::ParameterType::PARAMETER_INTEGER) {
            std::cout << "sensor mode changing" << std::endl;
            sensorMode = parameter.as_int();
        } else if (parameter.get_name() == "max_distance" &&
                   parameter.get_type() ==
                       rclcpp::ParameterType::PARAMETER_DOUBLE) {
            miradarParam.maxDistance =
                static_cast<int>(parameter.as_double() * 1000);
            double value = parameter.as_double();
            RCLCPP_INFO(node->get_logger(), "max distance set");
            RCLCPP_INFO(node->get_logger(), "distance : %f, converted : %d",
                        parameter.as_double(), miradarParam.maxDistance);
        } else if (parameter.get_name() == "min_distance" &&
                   parameter.get_type() ==
                       rclcpp::ParameterType::PARAMETER_DOUBLE) {
            miradarParam.minDistance =
                static_cast<int>(parameter.as_double() * 1000.0);
        } else if (parameter.get_name() == "alarm_distance" &&
                   parameter.get_type() ==
                       rclcpp::ParameterType::PARAMETER_DOUBLE) {
            miradarParam.alarmDistance =
                static_cast<int>(parameter.as_double() * 1000.0);
        } else if (parameter.get_name() == "max_dB" &&
                   parameter.get_type() ==
                       rclcpp::ParameterType::PARAMETER_INTEGER) {
            std::cout << parameter.as_int() << std::endl;
            miradarParam.maxDb = parameter.as_int();
        } else if (parameter.get_name() == "min_dB" &&
                   parameter.get_type() ==
                       rclcpp::ParameterType::PARAMETER_INTEGER) {
            miradarParam.minDb = parameter.as_int();
        } else if (parameter.get_name() == "distance_div" &&
                   parameter.get_type() ==
                       rclcpp::ParameterType::PARAMETER_INTEGER) {
            miradarParam.nDistance = parameter.as_int();
        } else if (parameter.get_name() == "angle_div" &&
                   parameter.get_type() ==
                       rclcpp::ParameterType::PARAMETER_INTEGER) {
            miradarParam.nAngle = parameter.as_int();
        } else if (parameter.get_name() == "tx_power" &&
                   parameter.get_type() ==
                       rclcpp::ParameterType::PARAMETER_INTEGER) {
            miradarParam.txPower = parameter.as_int();
        } else if (parameter.get_name() == "hpf_gain" &&
                   parameter.get_type() ==
                       rclcpp::ParameterType::PARAMETER_INTEGER) {
            miradarParam.hpfGain = parameter.as_int();
        } else if (parameter.get_name() == "pga_gain" &&
                   parameter.get_type() ==
                       rclcpp::ParameterType::PARAMETER_INTEGER) {
            miradarParam.pgaGain = parameter.as_int();
        } else if (parameter.get_name() == "duration" &&
                   parameter.get_type() ==
                       rclcpp::ParameterType::PARAMETER_INTEGER) {
            miradarParam.duration = parameter.as_int();
        } else if (parameter.get_name() == "scan_mode" &&
                   parameter.get_type() ==
                       rclcpp::ParameterType::PARAMETER_INTEGER) {
            laserscanMode = parameter.as_int();
        }
    }

    explicit MiRadarROS2()
        : sensorMode(1), isConfigUpdate(false), laserscanMode(1) {
        node = rclcpp::Node::make_shared("miradar_node");
        pub = node->create_publisher<miradar_msgs::msg::PPIData>(
            "/miradar/ppidata", 20);
        mapPub = node->create_publisher<sensor_msgs::msg::Image>(
            "/miradar/image_raw", 20);
        laserPub = node->create_publisher<sensor_msgs::msg::LaserScan>(
            "/miradar/scan", 20);
        pcPub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/miradar/points", 20);
        declareParams();

        deviceFile =
            (deviceFile.find("/dev/tty") == -1) ? "/dev/ttyUSB0" : deviceFile;

        initConnection();

        node->set_on_parameters_set_callback(
            std::bind(&MiRadarROS2::setParam, this, std::placeholders::_1));
    }

    void declareParams() {
        node->declare_parameter("devicename", "/dev/ttyUSB0");
        node->declare_parameter("sensor_mode", 1);
        node->declare_parameter("max_distance", 3);
        node->declare_parameter("min_distance", 0.3);
        node->declare_parameter("max_angle", 45);
        node->declare_parameter("max_dB", -20);
        node->declare_parameter("min_dB", -40);

        node->declare_parameter("alarm_distance", 0);
        node->declare_parameter("distance_div", 64);
        node->declare_parameter("angle_div", 45);
        node->declare_parameter("tx_power", -7);
        node->declare_parameter("hpf_gain", 1);
        node->declare_parameter("pga_gain", 1);
        node->declare_parameter("duration", 200);
        node->declare_parameter("scan_mode", 0);
    }

    void initConnection() {
        int fd;

        fd = serial.CommInit(deviceFile);
        if (fd < 0) {
            RCLCPP_INFO(node->get_logger(), "device open failed.");
            exit(-1);
        }
        miradar.setSerial(serial);
        RCLCPP_INFO(node->get_logger(), "Connected to %s",
                    (char *)deviceFile.c_str());
        miradar.setParam();
    }

    void changeSensorState() {
        RCLCPP_INFO(node->get_logger(), "changing");
        miradar.setSensorState(sensorMode);
    }

    void publishMap() {
        if (miradar.nAngle * miradar.nDistance == miradar.map.size()) {
            auto image = sensor_msgs::msg::Image();
            image.height = miradarParam.nAngle;
            image.width = miradarParam.nDistance;
            image.step = image.width;
            image.header.frame_id = "miradar";
            image.encoding = "mono8";
            std::copy(miradar.map.begin(), miradar.map.end(),
                      std::back_inserter(image.data));
            mapPub->publish(image);
        } else {
            RCLCPP_INFO(node->get_logger(), "map is corrupt. %d size",
                        miradar.map.size());
        }
    }

    void checkForEmptyParam() {
        miradarParam.maxAngle = (miradarParam.maxAngle == 0)
                                    ? miradar.radarParam.maxAngle
                                    : miradarParam.maxAngle;
        miradarParam.minDistance = (miradarParam.minDistance == 0)
                                       ? miradar.radarParam.minDistance
                                       : miradarParam.minDistance;
        miradarParam.maxDistance = (miradarParam.maxDistance == 0)
                                       ? miradar.radarParam.maxDistance
                                       : miradarParam.maxDistance;
        miradarParam.nAngle =
            (miradarParam.nAngle == 0) ? miradar.nAngle : miradarParam.nAngle;
        miradarParam.nDistance = (miradarParam.nDistance == 0)
                                     ? miradar.nDistance
                                     : miradarParam.nDistance;
    }

    void publishLaserScan() {
        RCLCPP_INFO(node->get_logger(), "%d, %d", miradarParam.nAngle,
                    miradarParam.nDistance);
        RCLCPP_INFO(node->get_logger(), "%d, %d", miradar.radarParam.nAngle,
                    miradar.radarParam.nDistance);

        if (miradarParam.nAngle * miradarParam.nDistance ==
            miradar.map.size()) {
            auto ls = sensor_msgs::msg::LaserScan();
            ls.header.frame_id = "miradar_scan";
            ls.angle_min = DEG2RAG(-static_cast<double>(miradarParam.maxAngle));
            ls.angle_max = DEG2RAG(static_cast<double>(miradarParam.maxAngle));
            ls.range_min =
                static_cast<double>(miradarParam.minDistance) / 1000.0;
            ls.range_max =
                static_cast<double>(miradarParam.maxDistance) / 1000.0;
            ls.angle_increment = (ls.angle_max - ls.angle_min) /
                                 static_cast<double>(miradarParam.nAngle);
            ls.time_increment = 0.001;
            float a = (ls.range_max - ls.range_min) /
                      (static_cast<double>(miradarParam.nDistance - 1));
            RCLCPP_INFO(node->get_logger(), "nAngle : %d", miradarParam.nAngle);

            for (int i = 0; i < miradarParam.nAngle; i++) {
                int max = -1;
                float distance = -1;
                float intensity = -1;

                for (int j = 0; j < miradarParam.nDistance; j++) {
                    if (laserscanMode == 0) {
                        if (MiRadar::pixel2DB(
                                miradar.map[i * miradarParam.nDistance + j]) >
                            miradarParam.minDb) {
                            distance = a * j + ls.range_min;
                            intensity =
                                static_cast<double>(
                                    miradar
                                        .map[i * miradarParam.nDistance + j]) /
                                255.0;
                            ls.ranges.push_back(distance);
                            ls.intensities.push_back(intensity);
                            break;
                        } else if (j == miradarParam.nDistance - 1) {
                            ls.ranges.push_back(999);
                            ls.intensities.push_back(0);
                        }
                    } else {
                        if (miradar.map[i * miradarParam.nDistance + j] > max) {
                            max = miradar.map[i * miradarParam.nDistance + j];
                            distance = a * j + ls.range_min;
                            intensity =
                                static_cast<double>(
                                    miradar
                                        .map[i * miradarParam.nDistance + j]) /
                                255.0;
                        }
                    }
                }
                if (laserscanMode == 1) {
                    ls.ranges.push_back(distance);
                    ls.intensities.push_back(intensity);
                }
            }
            ls.header.stamp = node->now();
            laserPub->publish(ls);
        }
    }

    void publishPointClouds() {
        pcl::PointCloud<pcl::PointXYZI> ladarpc;

        if (miradarParam.nAngle * miradarParam.nDistance ==
            miradar.map.size()) {
            auto pointcloud_msg = sensor_msgs::msg::PointCloud2();
            double angle_min =
                DEG2RAG(-static_cast<double>(miradarParam.maxAngle));
            double angle_max =
                DEG2RAG(static_cast<double>(miradarParam.maxAngle));
            double range_min =
                static_cast<double>(miradarParam.minDistance) / 1000.0;
            double range_max =
                static_cast<double>(miradarParam.maxDistance) / 1000.0;
            double angle_increment = (angle_max - angle_min) /
                                     static_cast<double>(miradarParam.nAngle);
            double distance_increment =
                (range_max - range_min) /
                static_cast<double>(miradarParam.nDistance);
            float a = (range_max - range_min) /
                      (static_cast<double>(miradarParam.nDistance - 1));
            float a_angle = (angle_max - angle_min) /
                            (static_cast<double>(miradarParam.nAngle - 1));

            for (int i = 0; i < miradarParam.nAngle; i++) {
                int max = -1;
                float distance = -1;
                float intensity = -1;
                float angle = 0;

                for (int j = 0; j < miradarParam.nDistance; j++) {
                    if (MiRadar::pixel2DB(
                            miradar.map[i * miradarParam.nDistance + j]) >
                        miradarParam.minDb) {
                        distance = a * j + range_min;
                        angle = a_angle * i + angle_min + angle_increment / 2;
                        intensity =
                            static_cast<double>(
                                miradar.map[i * miradar.radarParam.nDistance +
                                            j]) /
                            255.0;
                        double y = distance * sin(angle);
                        double x = distance * cos(angle);
                        pcl::PointXYZI point;
                        point.x = x;
                        point.y = y;
                        point.z = 0;
                        point.intensity = intensity;
                        ladarpc.push_back(point);
                    }
                }
            }
            pcl::toROSMsg(ladarpc, pointcloud_msg);
            pointcloud_msg.header.frame_id = "miradar";
            pcPub->publish(pointcloud_msg);
        }
    }

    void publishPPI() {
        if (miradar.ppiEntries.size() > 0) {
            auto ppidata = miradar_msgs::msg::PPIData();

            for (int i = 0; i < miradar.ppiEntries.size(); i++) {
                auto ppi = miradar_msgs::msg::PPI();
                double distance =
                    static_cast<double>(miradar.ppiEntries[i].distance) /
                    1000.0;
                double rad =
                    DEG2RAG(static_cast<double>(miradar.ppiEntries[i].angle));
                ppi.position.y = distance * sin(rad);
                ppi.position.x = distance * cos(rad);
                ppi.position.z = 0;

                ppi.speed = miradar.ppiEntries[i].speed;
                ppi.db = miradar.ppiEntries[i].db;
                ppidata.data.push_back(ppi);
            }
            pub->publish(ppidata);
        }
    }

    std::shared_ptr<rclcpp::Node> node;

    void run() {
        miradar.run();
        if (sensorMode == 1) {
            publishPPI();
        } else if (sensorMode == 2) {
            publishMap();
            publishLaserScan();
            publishPointClouds();
        }

        if (isConfigUpdate) {
            changeSensorState();
            miradar.setParam(miradarParam);
            isConfigUpdate = false;
        }
    }

private:
    Serial serial;
    MiRadarParam miradarParam;
    bool isConfigUpdate;
    MiRadar miradar;
    std::string deviceFile;
    int sensorMode;
    int laserscanMode;

    rclcpp::Publisher<miradar_msgs::msg::PPIData>::SharedPtr pub;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserPub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mapPub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcPub;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    MiRadarROS2 miradarROS;

    while (rclcpp::ok()) {
        miradarROS.run();
        rclcpp::spin_some(miradarROS.node);
    }
}
