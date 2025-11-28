/*
 * perception_node.cpp
 */
#include "autonomous_driving_config.hpp"
#include "perception_node.hpp"

using namespace Eigen;
using namespace std;

PerceptionNode::PerceptionNode(const std::string &node_name, const rclcpp::NodeOptions &options) : Node(node_name, options) {

    //QoS init 
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    //===============parameters===============
    //declare parameters(파라미터 등록+초기값 설정)
    this->declare_parameter("autonomous_driving/ns", "");
    this->declare_parameter("autonomous_driving/loop_rate_hz", 100.0);
    this->declare_parameter("autonomous_driving/use_manual_inputs", false);

    ProcessParams();

    RCLCPP_INFO(this->get_logger(), "vehicle_namespace: %s", cfg_.vehicle_namespace.c_str());
    RCLCPP_INFO(this->get_logger(), "loop_rate_hz: %f", cfg_.loop_rate_hz);
    RCLCPP_INFO(this->get_logger(), "use_manual_inputs: %d", cfg_.use_manual_inputs);

    // get parameters(파라미터 값 가져오기)

    // ROI
    //this->get_parameter("autonomous_driving/roi_front_length", cfg_.param_m_roi_front_length);
    //this->get_parameter("autonomous_driving/roi_rear_length", cfg_.param_m_roi_rear_length);
    //this->get_parameter("autonomous_driving/roi_left_width", cfg_.param_m_roi_left_width);
    //this->get_parameter("autonomous_driving/roi_right_width", cfg_.param_m_roi_right_width);

    // Reference Path
    //this->get_parameter("autonomous_driving/ref_csv_path", cfg_.param_ref_csv_path);

    //===========subscriber init===============
    //(1)s_manual_input_
    s_manual_input_ = 
    this->create_subscription<ad_msgs::msg::VehicleCommand>(
        "/manual_input", qos_profile, std::bind(&PerceptionNode::CallbackManualInput, this, std::placeholders::_1));

    //(2) s_vehicle_state_
    s_vehicle_state_ = 
    this->create_subscription<ad_msgs::msg::VehicleState>(
        "vehicle_state", qos_profile, std::bind(&PerceptionNode::CallbackVehicleState, this, std::placeholders::_1));

    //(3) s_lane_points_
    s_lane_points_ = 
    this->create_subscription<ad_msgs::msg::LanePointData>(
        "lane_points", qos_profile, std::bind(&PerceptionNode::CallbackLanePoints, this, std::placeholders::_1));

    //(4) s_mission_
    //s_mission_ = 
    //this->create_subscription<ad_msgs::msg::Mission>(
    //    "mission", qos_profile, std::bind(&PerceptionNode::CallbackMission, this, std::placeholders::_1));

    //===========publisher init===============
    //p_vehicle_command_ = 
    //this->create_publisher<ad_msgs::msg::VehicleCommand>(
    //    "vehicle_command", qos_profile);
    p_driving_way_= 
    this->create_publisher<ad_msgs::msg::PolyfitLaneData>(
        "driving_way", qos_profile);

    p_poly_lanes_ = 
    this->create_publisher<ad_msgs::msg::PolyfitLaneDataArray>(
        "poly_lanes", qos_profile);

    // Timer init
    t_run_node_ = this->create_wall_timer(
        std::chrono::milliseconds((int64_t)(1000 / cfg_.loop_rate_hz)),
        [this]() { this->Run(); });
}

PerceptionNode::~PerceptionNode() {}

void PerceptionNode::ProcessParams() {
    this->get_parameter("autonomous_driving/ns", cfg_.vehicle_namespace);
    this->get_parameter("autonomous_driving/loop_rate_hz", cfg_.loop_rate_hz);
    this->get_parameter("autonomous_driving/use_manual_inputs", cfg_.use_manual_inputs);
    ////////////////////// TODO //////////////////////
}

void PerceptionNode::Run() {
    //===================================================
    // Get subscribe variables 
    //일종의 input데이터 수집 단계 (멤버 변수 -> 지역변수로 복사 (mutex로 보호))
    //===================================================
    if (cfg_.use_manual_inputs == true) {
        if (b_is_manual_input_ == false) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Wait for Manual Input ...");
            return;
        }
    }

    if (b_is_simulator_on_ == false) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Wait for Vehicle State ...");
        return;
    }

    if (b_is_lane_points_ == false) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Wait for Lane Points ...");
        return;
    }

    //if (b_is_mission_ == false) {
    //    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Wait for Mission ...");
    //    return;
    //}

    // 일종의 input데이터 수집 단계 (멤버 변수 -> 지역변수로 복사 (mutex로 보호))

    interface::VehicleCommand manual_input; {
        if (cfg_.use_manual_inputs == true) {
            std::lock_guard<std::mutex> lock(mutex_manual_input_);
            manual_input = i_manual_input_;
        }
    }

    interface::VehicleState vehicle_state; {
        std::lock_guard<std::mutex> lock(mutex_vehicle_state_);
        vehicle_state = i_vehicle_state_;
    }
    interface::Lane lane_points; {
        std::lock_guard<std::mutex> lock(mutex_lane_points_);
        lane_points = i_lane_points_;
    }

    //interface::Mission mission; {
    //    std::lock_guard<std::mutex> lock(mutex_mission_);
    //    mission = i_mission_;
    //}

    //===================================================
    // Algorithm
    //===================================================
    interface::PolyfitLanes poly_lanes;

    interface::PolyfitLane driving_way = FindDrivingWay(vehicle_state, lane_points);
    //if (cfg_.use_manual_inputs == true) {
    //    vehicle_command = manual_input;
    //}

    //===================================================
    // Publish output
    //===================================================
    // (1) vehicle command
    //p_vehicle_command_->publish(ros2_bridge::UpdateVehicleCommand(vehicle_command));
    // (2) driving way
    p_driving_way_->publish(ros2_bridge::UpdatePolyfitLane(driving_way));
    // (3) polyfit lanes
    p_poly_lanes_->publish(ros2_bridge::UpdatePolyfitLanes(poly_lanes));
}

//===================================================
// [다훈 수정]FindDrivingWay 함수 구현
//===================================================
interface::PolyfitLane PerceptionNode::FindDrivingWay(const interface::VehicleState &vehicle_state, const interface::Lane& lane_points) {
    
    interface::PolyfitLane driving_way;
    driving_way.frame_id = lane_points.frame_id;

    /**
     * @brief Find the driving way from the lane points
     * inputs: vehicle_state, lane_points
     * outputs: driving_way_raw
     * Purpose: Implement lane fitting algorithm to find the driving way (center line) from the given lane points
     */

    ////////////////////// TODO //////////////////////

    /**********   Step 1: Separate lane points into left and right lanes ***********/
    // (This is a placeholder; actual polynomial fitting code should be implemented here)

    /* --------------------------------------------
    1-1 Check and initialize the number of points
    lane_points 구조 파악 (1): point 배열 
    practice 코드 그대로 
    -----------------------------------------------*/
    int num_points = lane_points.point.size(); // point 개수 파악

    /* --------------------------------------------
    1-2 Count left and right points
    point 메시지는 ros2표준 geometry_msgs/Point 타입의 배열임 (x,y,z) 
    좌표축 설정: x: 차량 진행방향, y: 차량 좌우방향(왼쪽이 +), z: 수직방향(위가 +)
    -----------------------------------------------*/
    int num_left_points = 0;
    int num_right_points = 0;
    for (int i=0; i<num_points; i++) {
        if (lane_points.point[i].y > 0) { // y>0 이면 왼쪽 차선
            num_left_points++;
        } else { // y<=0 이면 오른쪽 차선
            num_right_points++;
        }
    }

    /*--------------------------------------------
    1-3 Initialize X, Y, and A matrices for left and right lanes with correct sizes
    3차 다항식 fitting을 해야하니까 계수 4개 필요 (a,b,c,d)
    X행렬: 각 포인트의 x좌표를 다항식 형태로 변환 포인트x4 
    Y행렬: 각 포인트의 y좌표 포인트x1 
    Eigen 행렬 사용 
    typedef matrix<double, Dynamic, Dynamic> MatrixXd; 
    Vector는 1row or 1column matrix일 때 사용
    typedef matrix<int, Dynamic, 1> VectorXi;
    ----------------------------------------------*/
    //left lane
    Eigen::MatrixXd X_left(num_left_points, 4); // num_left_points x 4 (각 num_left_points 당 1x4)
    Eigen::VectorXd Y_left(num_left_points);    // num_left_points x 1 (각 num_left_points 당 1x1)
    //right lane
    Eigen::MatrixXd X_right(num_right_points, 4); // num_right_points x 4 (각 num_right_points 당 1x4)
    Eigen::VectorXd Y_right(num_right_points);    // num_right_points x 1 (각 num_right_points 당 1x1)


    /**********   Step 2: Fit a polynomial to each lane's points *******************/
    // (This is a placeholder; actual polynomial fitting code should be implemented here)
    /*least squares method 사용해서 y=ax^3 + bx^2 + cx + d 형태로 fitting

    Y = X × A

    Y = | y₁ |    X = | 1  x₁   x₁²  x₁³ |   A = | a |
        | y₂ |        | 1  x₂   x₂²  x₂³ |       | b |
        | y₃ |        | 1  x₃   x₃²  x₃³ |       | c |
        | .. |        | ...  ...  ... .. |       | d |

    A = (Xᵀ × X)⁻¹ × Xᵀ × Y 공식 */

    // [다훈 수정]2-0 X, Y 행렬 채우기
    int left_index = 0;
    int right_index = 0;
    for (int i=0; i<num_points; i++) {
        double x = lane_points.point[i].x;
        double y = lane_points.point[i].y;
        //left lane
        if (y>0){
            X_left(left_index, 0) = 1;
            X_left(left_index, 1) = pow(x, 1);
            X_left(left_index, 2) = pow(x, 2);
            X_left(left_index, 3) = pow(x, 3);
            Y_left(left_index) = y;
            left_index++;
        }
        //right lane
        else if (y<=0){
            X_right(right_index, 0) = 1;
            X_right(right_index, 1) = pow(x, 1);
            X_right(right_index, 2) = pow(x, 2);
            X_right(right_index, 3) = pow(x, 3);
            Y_right(right_index) = y;
            right_index++;
        }
    }

    // 2-1 Get optimized left lane coefficients A_left
    Eigen::VectorXd A_left = (X_left.transpose() * X_left).inverse() * X_left.transpose() * Y_left;

    // 2-2 Get optimized right lane coefficients A_right
    Eigen::VectorXd A_right = (X_right.transpose() * X_right).inverse() * X_right.transpose() * Y_right;


    /* Step 3: Determine the driving way as the center line between the left and right lane
            (This is a placeholder; actual center line calculation code should be implemented here)
            Update driving_way_raw with calculated center line
    left lane: y_left = a_L·x³ + b_L·x² + c_L·x + d_L
    right lane: y_right = a_R·x³ + b_R·x² + c_R·x + d_R
    center line: y_center = (y_left + y_right) / 2
    */

    //center line 계수 벡터 생성
    Eigen::VectorXd A_center(4);
    //3-1각 계수를 좌/우 평균으로 계산
    A_center(0) = (A_left(0) + A_right(0)) / 2;
    A_center(1) = (A_left(1) + A_right(1)) / 2;
    A_center(2) = (A_left(2) + A_right(2)) / 2;
    A_center(3) = (A_left(3) + A_right(3)) / 2;
    //3-2 driving_way_raw 메시지에 계수 저장 (PolyfitLaneData 메시지에 맞게 (a0~a3) 저장)
    driving_way.a0 = A_center(0);  // 상수항
    driving_way.a1 = A_center(1);  // x의 계수
    driving_way.a2 = A_center(2);  // x²의 계수
    driving_way.a3 = A_center(3);  // x³의 계수
    ////////////////////////////////////////////////////
    return driving_way;
}


int main(int argc, char **argv) {
    std::string node_name = "perception_node";

    // Initialize node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PerceptionNode>(node_name));
    rclcpp::shutdown();
    return 0;
}
