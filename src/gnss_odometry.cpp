#include <glim/odometry/callbacks.hpp>
#include <glim/mapping/callbacks.hpp>
#include <glim/util/extension_module_ros2.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/config.hpp>
#include <glim/util/concurrent_vector.hpp>
#include <glim/util/convert_to_string.hpp>


#include <gtsam_points/optimizers/isam2_result_ext.hpp>
#include <gtsam_points/optimizers/isam2_ext.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>


#include <memory>
#include <string>
#include <iostream>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>

#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

//#include "gnss_conversions.hpp"

#define GLIM_ROS2 //not sure if this is required. Delete it to test what it is doing

using NavSatFix = sensor_msgs::msg::NavSatFix;
using NavSatFixConstPtr = sensor_msgs::msg::NavSatFix::ConstSharedPtr;

//To quickly convert from stamp sec and nsec to a single double
template <typename Stamp>
double to_sec(Stamp& stamp) {
  return stamp.sec + stamp.nanosec / 1e9;
}

struct GNSS
{
  double stamp;
  Eigen::Vector3d position;        // x, y, z
  Eigen::Matrix3d covariance;      // position covariance
};


using namespace glim;
using gtsam::symbol_shorthand::X;

class GnssExtensionModule : public ExtensionModuleROS2 {
public:
  GnssExtensionModule()
    : logger(create_module_logger("gnss_extension"))
  {

    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;
    using std::placeholders::_4;
    
    GlobalMappingCallbacks::on_smoother_update.add(std::bind(&GnssExtensionModule::on_smoother_update_global, this, _1, _2, _3));
    GlobalMappingCallbacks::on_insert_submap.add(std::bind(&GnssExtensionModule::on_insert_submap, this, _1));
    GlobalMappingCallbacks::on_smoother_update_result.add(std::bind(&GnssExtensionModule::on_smoother_update_result, this, _1, _2));


    // TODO: Add a config file thing so I can easily pass parameters.

    // TODO: Maybe re-fine GNSS->IMU pose online?

    logger->info("GNSS extension module initialized");

    kill_switch = false;
    thread = std::thread([this] { backend_task(); });
  }

  ~GnssExtensionModule(){
    // Write the T_world_ecef to a file
    kill_switch = true;
    thread.join();
  }

  std::vector<GenericTopicSubscription::Ptr>
  create_subscriptions() override
  {
    const auto sub =
      std::make_shared<TopicSubscription<NavSatFix>>(
        "/fix",
        [this](const NavSatFixConstPtr msg) {
          gnss_callback(msg);
        }
      );

    return {sub};
  }

    void on_smoother_update_global(gtsam_points::ISAM2Ext& isam2, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_values){

        const auto factors = output_factors.get_all_and_clear();
        if (!factors.empty()) {
            new_factors.add(factors.back());
            current_factor = factors.back();
            //new_factors.print();
        }

    }

    void on_smoother_update_result(gtsam_points::ISAM2Ext& isam2, const gtsam_points::ISAM2ResultExt& result) {

// TODO: Understand how I can warn if the GNSS factors are not aligning with the rest of factors.
//        if (ecef2local_initialized){
//
//            if (isam2.valueExists(X(current_submap->id))) {
//                
//                logger->info("The result thing for node {}: ", current_submap->id);
//                
//                auto info_matrix = isam2.marginalFactor(X(current_submap->id))->augmentedInformation();
//                auto cov = isam2.marginalCovariance(X(current_submap->id));
//                std::cout << " " << info_matrix << " " << std::endl;
//
//            }
//
//        }

    }


    void on_insert_submap(const SubMap::ConstPtr& submap) { 
        input_submap_queue.push_back(submap); 
    }

  private:

    std::shared_ptr<spdlog::logger> logger;
    std::atomic_bool kill_switch;
    std::thread thread;

    // Geodetic system parameters
    double kSemimajorAxis = 6378137;
    double kSemiminorAxis = 6356752.3142;
    double kFirstEccentricitySquared = 6.69437999014 * 0.001;
    double kSecondEccentricitySquared = 6.73949674228 * 0.001;
    double kFlattening = 1 / 298.257223563;
    double baseline_min = 5; // needs to walk 5m for the GNSS to compute the T_world_ecef
    
    double kMaxTimeDiff = 0.05; // 50 ms 


    SubMap::ConstPtr current_submap;
    gtsam::NonlinearFactor::shared_ptr current_factor;


    bool ecef2local_initialized = false;

    ConcurrentVector<GNSS> input_gnss_queue;
    ConcurrentVector<SubMap::ConstPtr> input_submap_queue;

    ConcurrentVector<gtsam::NonlinearFactor::shared_ptr> output_factors;
    Eigen::Isometry3d T_world_ecef;


    void backend_task(){

        std::deque<GNSS> gnss_queue;
        std::deque<SubMap::ConstPtr> submap_queue;

        while (!kill_switch)
        {

            std::deque<GNSS> gnss_matched;
            std::deque<SubMap::ConstPtr> submap_matched;

            const auto gnss_data = input_gnss_queue.get_all_and_clear();
            const auto new_submaps = input_submap_queue.get_all_and_clear();
            gnss_queue.insert(gnss_queue.end(), gnss_data.begin(), gnss_data.end());
            submap_queue.insert(submap_queue.end(), new_submaps.begin(), new_submaps.end());

            // Remove submaps that are younger than the most recent GNSS message;
            if(!gnss_queue.empty() && !submap_queue.empty()){
                while (submap_queue.front()->frames.front()->stamp < gnss_queue.front().stamp){
                    submap_queue.pop_front();
                }
            }

            //Align GNSS with map measurements
            size_t i = 0; // submaps
            size_t j = 0; // gnss

            while (i < submap_queue.size() && j < gnss_queue.size()) {
                double t_sub = submap_queue[i]->frames.front()->stamp;
                double t_gnss = gnss_queue[j].stamp;
            
                double dt = t_sub - t_gnss;
            
                if (std::abs(dt) < kMaxTimeDiff) {
                    // MATCH FOUND
                    submap_matched.push_back(submap_queue[i]);
                    gnss_matched.push_back(gnss_queue[j]);

                    ++i;
                    ++j;
                }
                else if (dt > 0) {
                    // GNSS too old
                    ++j;
                }
                else {
                    // Submap too old
                    ++i;
                }
            }


            //Compute the T_world_ecef in case it does not exist yet!
            if(!ecef2local_initialized && !submap_matched.empty() && !gnss_matched.empty()){
                //Check if the distance between first and last is greater than the min_baseline

                double baseline = (submap_matched.front()->T_world_origin.inverse() * submap_matched.back()->T_world_origin).translation().norm();

                if (baseline >= baseline_min){
                    logger->info("About to initialize T_world_ecef.");
                    Eigen::MatrixXd X(3, submap_matched.size());
                    Eigen::MatrixXd Y(3, gnss_matched.size());

                    for (size_t i = 0; i < gnss_matched.size(); ++i) {
                        X.col(i) = submap_matched[i]->T_world_origin.translation();
                        Y.col(i) = gnss_matched[i].position;
                    }

                    Eigen::Matrix4d T_mat = Eigen::umeyama(X, Y, false); //Umeyama's transform to align 2 trajectories

                    Eigen::Isometry3d T_ecef_world(T_mat);
                    T_world_ecef = T_ecef_world.inverse();


                    ecef2local_initialized = true;

                }
            }

            if (ecef2local_initialized) {

                std::deque<GNSS> gnss_queue;
                std::deque<SubMap::ConstPtr> submap_queue;

                const auto& submap = submap_matched.back();
                const auto& gnss = gnss_matched.back();

                Eigen::Vector3d xyz = T_world_ecef * gnss.position;

                Eigen::Matrix3d cov = gnss.covariance;

                auto gaussian = gtsam::noiseModel::Gaussian::Covariance(cov);

                // TODO: Create a parameter to control the Huber param
                auto robust = gtsam::noiseModel::Robust::Create(
                                gtsam::noiseModel::mEstimator::Huber::Create(1.345),
                                gaussian
                            );

                //TODO: Create a GNSS Factor Instead of PosePrior AND connect it to the IMU node, not submap->id 
                gtsam::NonlinearFactor::shared_ptr factor(new gtsam::PoseTranslationPrior<gtsam::Pose3>(X(submap->id), xyz, robust));
                current_submap = submap;
                output_factors.push_back(factor);
            }


        }
        

    }

    void gnss_callback(const NavSatFixConstPtr msg)
    {
        GNSS gnss_msg;
        gnss_msg.stamp = to_sec(msg->header.stamp);
        gnss_msg.position = wgs84_to_ecef(msg->latitude, msg->longitude, msg->altitude);

        gnss_msg.covariance <<  msg->position_covariance[0], 
                                msg->position_covariance[1],
                                msg->position_covariance[2],
                                msg->position_covariance[3],
                                msg->position_covariance[4],
                                msg->position_covariance[5],
                                msg->position_covariance[6],
                                msg->position_covariance[7],
                                msg->position_covariance[8];

        if(!ecef2local_initialized){
            auto gnss_debug = T_world_ecef * gnss_msg.position;
            logger->debug("[ECEF] GNSS Reported Position (x,y,z): ({},{},{})", gnss_debug.x(), gnss_debug.y(), gnss_debug.z());
        }


        input_gnss_queue.push_back(gnss_msg);

    }

    Eigen::Vector3d ecef_to_wgs84(const Eigen::Vector3d& xyz) {
      // Convert ECEF coordinates to geodetic coordinates.
      // J. Zhu, "Conversion of Earth-centered Earth-fixed coordinates
      // to geodetic coordinates," IEEE Transactions on Aerospace and
      // Electronic Systems, vol. 30, pp. 957-961, 1994.

      const double x = xyz.x();
      const double y = xyz.y();
      const double z = xyz.z();

      double r = sqrt(x * x + y * y);
      double Esq = kSemimajorAxis * kSemimajorAxis - kSemiminorAxis * kSemiminorAxis;
      double F = 54 * kSemiminorAxis * kSemiminorAxis * z * z;
      double G = r * r + (1 - kFirstEccentricitySquared) * z * z - kFirstEccentricitySquared * Esq;
      double C = (kFirstEccentricitySquared * kFirstEccentricitySquared * F * r * r) / pow(G, 3);
      double S = cbrt(1 + C + sqrt(C * C + 2 * C));
      double P = F / (3 * pow((S + 1 / S + 1), 2) * G * G);
      double Q = sqrt(1 + 2 * kFirstEccentricitySquared * kFirstEccentricitySquared * P);
      double r_0 = -(P * kFirstEccentricitySquared * r) / (1 + Q) +
                   sqrt(0.5 * kSemimajorAxis * kSemimajorAxis * (1 + 1.0 / Q) - P * (1 - kFirstEccentricitySquared) * z * z / (Q * (1 + Q)) - 0.5 * P * r * r);
      double U = sqrt(pow((r - kFirstEccentricitySquared * r_0), 2) + z * z);
      double V = sqrt(pow((r - kFirstEccentricitySquared * r_0), 2) + (1 - kFirstEccentricitySquared) * z * z);
      double Z_0 = kSemiminorAxis * kSemiminorAxis * z / (kSemimajorAxis * V);

      const double alt = U * (1 - kSemiminorAxis * kSemiminorAxis / (kSemimajorAxis * V));
      const double lat = atan((z + kSecondEccentricitySquared * Z_0) / r) * 180.0 / M_PI;
      const double lon = atan2(y, x) * 180.0 / M_PI;

      return {lat, lon, alt};
    }

    Eigen::Vector3d wgs84_to_ecef(double lat, double lon, double alt) {
      double lat_rad = lat * M_PI / 180.0;
      double lon_rad = lon * M_PI / 180.0;
      double xi = sqrt(1 - kFirstEccentricitySquared * sin(lat_rad) * sin(lat_rad));
      const double x = (kSemimajorAxis / xi + alt) * cos(lat_rad) * cos(lon_rad);
      const double y = (kSemimajorAxis / xi + alt) * cos(lat_rad) * sin(lon_rad);
      const double z = (kSemimajorAxis / xi * (1 - kFirstEccentricitySquared) + alt) * sin(lat_rad);

      return {x, y, z};
    }

    Eigen::Isometry3d calc_T_ecef_nwz(const Eigen::Vector3d& ecef, double radius) {
      const Eigen::Vector3d z = ecef.normalized();
      const Eigen::Vector3d to_north = (Eigen::Vector3d::UnitZ() * radius - ecef).normalized();
      const Eigen::Vector3d x = (to_north - to_north.dot(z) * z).normalized();
      const Eigen::Vector3d y = z.cross(x);

      Eigen::Isometry3d T_ecef_nwz = Eigen::Isometry3d::Identity();
      T_ecef_nwz.linear().col(0) = x;
      T_ecef_nwz.linear().col(1) = y;
      T_ecef_nwz.linear().col(2) = z;
      T_ecef_nwz.translation() = ecef;

      return T_ecef_nwz;
    }

    Eigen::Isometry3d calc_T_ecef_enu(const Eigen::Vector3d& ecef) {

        const Eigen::Vector3d up = ecef.normalized();

        Eigen::Vector3d east = Eigen::Vector3d::UnitZ().cross(up);
        if (east.norm() < 1e-8) {
          east = Eigen::Vector3d::UnitX();
        } else {
          east.normalize();
        }

        const Eigen::Vector3d north = up.cross(east);

        Eigen::Isometry3d T_ecef_enu = Eigen::Isometry3d::Identity();
        T_ecef_enu.linear().col(0) = east;
        T_ecef_enu.linear().col(1) = north;
        T_ecef_enu.linear().col(2) = up;
        T_ecef_enu.translation() = ecef;

        return T_ecef_enu;
    }

    Eigen::Vector3d ecef_to_enu(
        const Eigen::Vector3d& ecef_point,
        const Eigen::Vector3d& ecef_origin)
    {
        // Build ENU frame at origin
        Eigen::Isometry3d T_ecef_enu = calc_T_ecef_enu(ecef_origin);

        const Eigen::Matrix3d R = T_ecef_enu.linear();
        const Eigen::Vector3d t = T_ecef_enu.translation();

        // Vector from origin to point in ECEF
        Eigen::Vector3d dp = ecef_point - t;

        // Express in ENU frame
        return R.transpose() * dp;
    }

    double harversine(const Eigen::Vector2d& latlon1, const Eigen::Vector2d& latlon2) {
      const double lat1 = latlon1[0];
      const double lon1 = latlon1[1];
      const double lat2 = latlon2[0];
      const double lon2 = latlon2[1];

      const double dlat = lat2 - lat1;
      const double dlon = lon2 - lon1;

      const double a = std::pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * std::pow(sin(dlon / 2), 2);
      const double c = 2 * atan2(sqrt(a), sqrt(1 - a));

      return kSemimajorAxis * c;
    }

};



extern "C" ExtensionModuleROS2* create_extension_module() {
  return new GnssExtensionModule();
}
