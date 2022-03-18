#include <aruco_ekf_slam/aruco_ekf_slam.h>

ArUcoEKFSLAM::ArUcoEKFSLAM():private_node("~")
{
    ROS_INFO_STREAM("\033[1;32m----> EKFSLAM started.\033[0m");
    GetTopicName();
    GetParameter();
    is_init_ = false;
    mu_.resize(3, 1);
    mu_.setZero();
    sigma_.resize(3, 3);
    sigma_.setZero();
    dictionary_ = aruco::generateCustomDictionary(n_markers_, marker_size_);

    image_sub = node_handle.subscribe(image_topic, 1, &ArUcoEKFSLAM::ImageCallback, this);
    encoder_sub = node_handle.subscribe(encoder_topic, 1, &ArUcoEKFSLAM::EncoderCallback, this);

    g_landmark_pub = node_handle.advertise<visualization_msgs::MarkerArray>("ekf_slam/landmark", 1, this);
    g_robot_pose_pub = node_handle.advertise<geometry_msgs::PoseWithCovarianceStamped>("ekf_slam/pose", 1, this);
    image_transport::ImageTransport it(node_handle);
    g_img_pub = it.advertise("ekf_slam/image", 1, this);
};

void ArUcoEKFSLAM::GetTopicName()
{
    node_handle.getParam("/slam_node/topic/encoder", encoder_topic);
    node_handle.getParam("/slam_node/topic/image", image_topic);
}

void ArUcoEKFSLAM::GetParameter()
{

    double fx, fy, cx, cy, k1, k2, p1, p2, k3;
    node_handle.getParam("/slam_node/camera/fx", fx);
    node_handle.getParam("/slam_node/camera/fy", fy);
    node_handle.getParam("/slam_node/camera/cx", cx);
    node_handle.getParam("/slam_node/camera/cy", cy);
    node_handle.getParam("/slam_node/camera/k1", k1);
    node_handle.getParam("/slam_node/camera/k2", k2);
    node_handle.getParam("/slam_node/camera/p1", p1);
    node_handle.getParam("/slam_node/camera/p2", p2);
    node_handle.getParam("/slam_node/camera/k3", k3);

    K_ = (cv::Mat_<float>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    dist_ = (cv::Mat_<float>(5, 1) << k1, k2, p1, p2, k3);

    node_handle.getParam("/slam_node/odom/kl", kl_);
    node_handle.getParam("/slam_node/odom/kr", kr_);
    node_handle.getParam("/slam_node/odom/b", b_);
    node_handle.getParam("/slam_node/covariance/k", k_);
    node_handle.getParam("/slam_node/covariance/k_r", k_r_);
    node_handle.getParam("/slam_node/covariance/k_phi", k_phi_);
    node_handle.getParam("/slam_node/aruco/n_markers", n_markers_);
    node_handle.getParam("/slam_node/aruco/marker_size", marker_size_);
    node_handle.getParam("/slam_node/aruco/marker_length", marker_length_);

    std::vector<double> Trc;
    node_handle.getParam("/slam_node/extrinsic/Trc", Trc);
    T_r_c_ << Trc[0], Trc[1], Trc[2], Trc[3],
        Trc[4], Trc[5], Trc[6], Trc[7],
        Trc[8], Trc[9], Trc[10], Trc[11],
        0.0, 0.0, 0.0, 1.0;
}

void ArUcoEKFSLAM::addEncoder(const double &enl, const double &enr)
{
    if (is_init_ == false)
    {
        last_enl_ = enl;
        last_enr_ = enr;
        is_init_ = true;
        return;
    }
    double delta_enl = enl - last_enl_;
    double delta_enr = enr - last_enr_;

    double delta_s_l = kl_ * delta_enl;
    double delta_s_r = kr_ * delta_enr;

    double delta_theta = (delta_s_r - delta_s_l) / b_;
    double delta_s = (delta_s_l + delta_s_r) / 2;

    double temp_angle = mu_(2, 0) + 0.5 * delta_theta;
    mu_(0, 0) += delta_s * cos(temp_angle);
    mu_(1, 0) += delta_s * sin(temp_angle);
    mu_(2, 0) += delta_theta;
    normAngle(mu_(2, 0));

    Matrix<double, 3, 2> G_u_p;
    G_u_p << 0.5 * (cos(temp_angle) - delta_s * sin(temp_angle) / b_), 0.5 * (cos(temp_angle) + delta_s * sin(temp_angle) / b_),
        0.5 * (sin(temp_angle) + delta_s * cos(temp_angle) / b_), 0.5 * (sin(temp_angle) - delta_s * cos(temp_angle) / b_),
        1.0 / b_, -1.0 / b_;
    Matrix<double, 3, 2> G_xi;
    G_xi << 1.0, 0.0, -delta_s * sin(temp_angle),
        0.0, 1.0, delta_s * cos(temp_angle),
        0.0, 0.0, 1.0;

    int N = mu_.rows();
    MatrixXd F(N, 3);
    F.setZero();
    F.block(0, 0, 3, 3) = Matrix3d::Identity();

    MatrixXd G_t = MatrixXd::Identity(N, N);
    G_t.block(0, 0, 3, 3) = G_xi;

    Matrix2d sigma_u;
    sigma_u << k_ * k_ * delta_s_r * delta_s_r, 0.0, 0.0, k_ * k_ * delta_s_l * delta_s_l;

    sigma_ = G_t * sigma_ * G_t.transpose() + F * G_u_p * sigma_u * G_u_p.transpose() * F.transpose();

    last_enl_ = enl;
    last_enr_ = enr;
}

void ArUcoEKFSLAM::addImage(const Mat &img)
{
    if (is_init_ == false)
        return;
    std::vector<Observation> obs;
    getObservations(img, obs);

    for (Observation ob : obs)
    {
        Eigen::Matrix2d Q;
        Q << k_r_ * k_r_ * fabs(ob.r_ * ob.r_), 0.0,
            0.0, k_phi_ * k_phi_ * fabs(ob.phi_ * ob.phi_);

        int i;
        // if the landmark already exists
        if (checkLandmark(ob.aruco_id_, i))
        {
            int N = mu_.rows();
            MatrixXd F(5, N);
            F.setZero();
            F.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
            F(3, 3 + 2 * i) = 1;
            F(4, 4 + 2 * i) = 1;

            double &m_x = mu_(3 + 2 * i, 0);
            double &m_y = mu_(4 + 2 * i, 0);
            double &x = mu_(0, 0);
            double &y = mu_(1, 0);
            double &theta = mu_(2, 0);

            double delta_x = m_x - x;
            double delta_y = m_y - y;
            double q = delta_x * delta_x + delta_y * delta_y;
            double sqrt_q = sqrt(q);
            MatrixXd H_v(2, 5);
            H_v << -sqrt_q * delta_x, -sqrt_q * delta_y, 0, sqrt_q * delta_x, sqrt_q * delta_y,
                delta_y, -delta_x, -q, -delta_y, delta_x;

            H_v = (1 / q) * H_v;

            MatrixXd Ht = H_v * F;

            MatrixXd K = sigma_ * Ht.transpose() * (Ht * sigma_ * Ht.transpose() + Q).inverse();

            double phi_hat = atan2(delta_y, delta_x) - theta;
            normAngle(phi_hat);
            Vector2d z_hat(sqrt_q, phi_hat);
            Vector2d z(ob.r_, ob.phi_);
            mu_ = mu_ + K * (z - z_hat);
            MatrixXd I = Eigen::MatrixXd::Identity(N, N);
            sigma_ = (I - K * Ht) * sigma_;
        }
        // else if the landmark doesn't exist
        else
        {
            double angle = mu_(2, 0) + ob.phi_;
            normAngle(angle);

            double m_x = ob.r_ * cos(angle) + mu_(0, 0);
            double m_y = ob.r_ * sin(angle) + mu_(1, 0);

            Matrix3d sigma_xi = sigma_.block(0, 0, 3, 3);

            Matrix<double, 2, 3> G_p;
            G_p << 1, 0, -ob.r_ * sin(angle),
                0, 1, ob.r_ * cos(angle);

            Matrix2d G_z;
            G_z << cos(angle), -ob.r_ * sin(angle),
                sin(angle), ob.r_ * cos(angle);

            Matrix2d sigma_m = G_p * sigma_xi * G_p.transpose() + G_z * Q * G_z.transpose();

            MatrixXd G_fx;
            G_fx.resize(2, mu_.rows());
            G_fx.setZero();
            G_fx.block(0, 0, 2, 3) = G_p;
            MatrixXd sigma_mx;
            sigma_mx.resize(2, mu_.rows());
            sigma_mx.setZero();
            sigma_mx = G_fx * sigma_;

            int N = mu_.rows();
            MatrixXd temp_mu_(N + 2, 1);
            temp_mu_.setZero();
            temp_mu_ << mu_, m_x, m_y;
            mu_.resize(N + 2, 1);
            mu_ = temp_mu_;

            MatrixXd temp_sigma(N + 2, N + 2);
            temp_sigma.setZero();
            temp_sigma.block(0, 0, N, N) = sigma_;
            temp_sigma.block(N, N, 2, 2) = sigma_m;
            temp_sigma.block(N, 0, 2, N) = sigma_mx;
            temp_sigma.block(0, N, N, 2) = sigma_mx.transpose();

            temp_sigma.resize(N + 2, N + 2);
            sigma_ = temp_sigma;

            aruco_ids_.push_back(ob.aruco_id_);
        }
    }
}

int ArUcoEKFSLAM::getObservations(const cv::Mat &img, std::vector<Observation> &obs)
{
    std::vector<std::vector<cv::Point2f>> marker_corners;
    std::vector<int> IDs;
    std::vector<cv::Vec3d> rvs, tvs;
    cv::aruco::detectMarkers(img, dictionary_, marker_corners, IDs);
    cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_length_, K_, dist_, rvs, tvs);

    /* draw all marks */
    marker_img_ = img.clone();
    cv::aruco::drawDetectedMarkers(marker_img_, marker_corners, IDs);
    for (size_t i = 0; i < IDs.size(); i++)
        cv::aruco::drawAxis(marker_img_, K_, dist_, rvs[i], tvs[i], 0.07);

    /*  筛选距离较近的使用 */
    const float DistTh = 3; //3 m
    for (size_t i = 0; i < IDs.size(); i++)
    {
        float dist = cv::norm<double>(tvs[i]); //计算距离
        if (dist > DistTh)
            continue;

        /* 转化一下成Eigen T */
        cv::Vec3d tvec = tvs[i];
        cv::Vec3d rvec = rvs[i];
        cv::Mat R;
        cv::Rodrigues(rvec, R);
        Eigen::Matrix4d T_c_m;
        T_c_m << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), tvec[0],
            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), tvec[1],
            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), tvec[2],
            0., 0., 0., 1.;
        Eigen::Matrix4d T_r_m = T_r_c_ * T_c_m;

        double &x = T_r_m(0, 3);
        double &y = T_r_m(1, 3);

        double r = sqrt(x * x + y * y);
        double phi = atan2(y, x);
        int aruco_id = IDs[i];

        /* 加入到观测vector */
        obs.push_back(Observation(aruco_id, r, phi));
    } //for all detected markers

    return obs.size();
}

void ArUcoEKFSLAM::ImageCallback(const sensor_msgs::ImageConstPtr &img_ptr)
{
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(img_ptr);
    addImage(cv_ptr->image);
    cv::Mat img = markedImg();
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    g_img_pub.publish(msg);
}

void ArUcoEKFSLAM::EncoderCallback(const geometry_msgs::QuaternionStamped::ConstPtr &encoder_ptr)
{
    double enl1 = encoder_ptr->quaternion.x;
    double enl2 = encoder_ptr->quaternion.y;
    double enr1 = encoder_ptr->quaternion.z;
    double enr2 = encoder_ptr->quaternion.w;

    double enl = 0.5 * (enl1 + enl2);
    double enr = 0.5 * (enr1 + enr2);

    addEncoder(enl, enr);
    visualization_msgs::MarkerArray marker = toRosMarker(4);
    g_landmark_pub.publish(marker);

    geometry_msgs::PoseWithCovarianceStamped pose = toRosPose();
    g_robot_pose_pub.publish(pose);
}

visualization_msgs::MarkerArray ArUcoEKFSLAM::toRosMarker(double scale)
{
    visualization_msgs::MarkerArray markers;
    int N = 0;
    for (int i = 4; i < mu_.rows(); i += 2)
    {
        double &mx = mu_(i - 1, 0);
        double &my = mu_(i, 0);

        /* 计算地图点的协方差椭圆角度以及轴长 */
        Eigen::Matrix2d sigma_m = sigma_.block(i - 1, i - 1, 2, 2); //协方差
        cv::Mat cvsigma_m = (cv::Mat_<double>(2, 2) << sigma_m(0, 0), sigma_m(0, 1), sigma_m(1, 0), sigma_m(1, 1));
        cv::Mat eigen_value, eigen_vector;
        cv::eigen(cvsigma_m, eigen_value, eigen_vector);
        double angle = atan2(eigen_vector.at<double>(0, 1), eigen_vector.at<double>(0, 0));
        double x_len = 2 * sqrt(eigen_value.at<double>(0, 0) * 5.991);
        double y_len = 2 * sqrt(eigen_value.at<double>(1, 0) * 5.991);

        /* 构造marker */
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time();
        marker.ns = "ekf_slam";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = mx;
        marker.pose.position.y = my;
        marker.pose.position.z = 0;
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
        marker.scale.x = scale * x_len;
        marker.scale.y = scale * y_len;
        marker.scale.z = 0.1 * scale * (x_len + y_len);
        marker.color.a = 0.8; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        markers.markers.push_back(marker);
    } // for all mpts

    return markers;
}
