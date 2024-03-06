#include <iostream>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <vector>
#include <list>
#include <array>
#include <mutex>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "ee4308_lib/core.hpp"
#include "ee4308_turtle/raytracer.hpp"
#include "ee4308_turtle/grid.hpp"
#include "ee4308_interfaces/srv/path_within_inflation.hpp"

#pragma once
namespace ee4308::turtle
{
    struct PlannerSmootherParameters
    { // contains defaults that can be overwritten
        struct Topics
        {
            std::string plan = "plan";
        } topics;
        struct Services
        {
            std::string get_inflation_layer = "get_inflation_layer";
            std::string get_plan = "get_plan";
            std::string check_path_within_inflation = "check_path_within_inflation"; // the service name to check if a path has crossed into a cell on the inflation layer that has a lethal inflation cost (i.e. is too close to an obstacle)
        } services;
        std::string frame_id = "map";
        double spline_vel = 0.2;
    };

    struct PlannerNode
    {
        PlannerNode *parent = nullptr;
        V2 cell = {0, 0};
        double cost_h = INFINITY;
        double cost_g = INFINITY;
        double cost_f = INFINITY;
        bool expanded = false;
    };

    /**
     * Implements the open list with insert sort.
     */
    class OpenList
    { // insert sort. reasonably simple and fast.
    private:
        std::list<PlannerNode *> list_;

    public:
        /**
         * Enqueues a node based on its f-cost.
         */
        void queue(PlannerNode *const &node)
        {
            auto node_it = list_.begin();
            for (; node_it != list_.end(); node_it = std::next(node_it))
                if (node->cost_f < (*node_it)->cost_f)
                    break;
            list_.insert(node_it, node);
        }

        /**
         * Polls the cheapest f-cost node.
         */
        PlannerNode *poll()
        {
            PlannerNode *front_node = list_.front();
            list_.pop_front();
            return front_node;
        }

        /**
         * Returns true if no more nodes.
         */
        bool empty() const { return list_.empty(); }

        /**
         * Clears the open list
         */
        void clear() { list_.clear(); }
    };

    /**
     * The PlannerSmoother class. Is Dijkstra. Convert to A*.
     * Additionally smooths the planner's path into a more feasible trajectory.
     */
    class PlannerSmoother
    {
    private:
        PlannerSmootherParameters params_;
        // RayTracer ray_tracer_; // for Theta*. Note that the center of cells have grid coordinates (##.5, ##.5).

        Costmap inflation_layer_;
        OpenList open_list_;

        std::vector<V2d> path_; // in world coordinates
        std::array<Relative, 8> neighbor_lut_;

    public:
        /**
         * Constructor for PlannerSmoother using PlannerSmootherParameters
         * @param params a struct containing all parameters for the Planner class.
         */
        PlannerSmoother(const PlannerSmootherParameters &params)
            : params_(params)
        {
            // Create a look-up table to store the relative coordinates, indices, and costs for the neighboring cells.
            for (int h = 0; h < 8; ++h)
            {
                Relative &neighbor = neighbor_lut_[h];
                neighbor.rel_cell = headingToDirection(h);
                // neighbor.rel_idx // not required.
                neighbor.value = (h & 1) == 0 ? 1 : std::sqrt(2); // if even, put 1. If odd, put sqrt(2).
            }
        }

    private:
        /**
         * Fills the path (in world coordinates) once the expanded node lies on the goal cell
         * The function will write at least the original goal coordinate: [goal_coord, ...],
         * followed by the world coordinates at the center of the cells that lie along the path.
         * @param expanded_node the expanded node
         * @param goal_coord the goal world coordinates
         */
        void foundPath(PlannerNode *expanded_node, const V2d &goal_coord)
        {
            PlannerNode *node = expanded_node;
            std::cout << "Path Found: { ";

            // fill the goal coord (based on the original coordinates)
            path_.push_back(goal_coord);
            std::cout << path_.back() << "; ";

            // fill the coords, if any
            do
            {
                path_.push_back(inflation_layer_.cellToWorld(node->cell, true));
                std::cout << path_.back() << "; ";
                node = node->parent;
            } while (node != nullptr);

            std::cout << "}" << std::endl;
        }

    public:
        /**
         * Finds a path between the start and goal world frame coordinates.
         * The returned path is *reversed*.
         * The goal point is at the beginning of the path, and the start point is at the back.
         * Assumes that the map is already updated with updateCostMap.
         * @param start_coord world coordinates to search from.
         * @param goal_coord world coordinates to search to.
         */
        const std::vector<V2d> &run(const V2d &start_coord, const V2d &goal_coord)
        { // changed to A*
            // clear path
            path_.clear();

            // Convert world coordinates to cell coordinates
            V2 start_cell = inflation_layer_.worldToCell(start_coord);
            V2 goal_cell = inflation_layer_.worldToCell(goal_coord);

            // Initialize nodes with the same size as the inflation layer
            std::vector<PlannerNode> nodes(inflation_layer_.size().x * inflation_layer_.size().y);

            { // Initialize the start node and queue into open list
                long idx = inflation_layer_.cellToIdx(start_cell);
                PlannerNode *node = &nodes[idx];
                node->cost_g = 0;
                // node->cost_h = 0; // FIX ME
                node->cost_f = 0;
                node->cell = start_cell;
                open_list_.queue(node);
            }

            // Loop until path is found or open_list is empty.
            while (open_list_.empty() == false && rclcpp::ok())
            {
                PlannerNode *const expanded_node = open_list_.poll();

                // skip if in closed list.
                if (expanded_node->expanded == true)
                    continue;
                expanded_node->expanded = true;

                // return path if expanded_node is the goal node
                if (expanded_node->cell == goal_cell)
                { // goal found. return path
                    foundPath(expanded_node, goal_coord);
                    break; // break to clear open_list
                }

                // search the neighbors
                for (const Relative &neighbor : neighbor_lut_)
                {
                    // get neighbor's cell coordinate
                    V2 neighbor_cell = expanded_node->cell + neighbor.rel_cell;

                    // skip neighbor node if out of map
                    if (inflation_layer_.outOfMap(neighbor_cell))
                        continue;

                    // get neighbor's index
                    long neighbor_idx = inflation_layer_.cellToIdx(neighbor_cell);

                    // get neighbor node.
                    PlannerNode *const neighbor_node = &nodes[neighbor_idx];

                    // skip if neighbor is already in closed list (has to be cheaper than expanded node)
                    if (neighbor_node->expanded == true)
                        continue;

                    // find the cost to reach the neighbor from the current node, with extra costs from the inflation layer.
                    double diff_g = neighbor.value * (inflation_layer_(neighbor_idx) + 1); // add 1 to avoid zero costs.
                    double test_g = expanded_node->cost_g + diff_g;

                    // queue neighbor node to open list and write data to if test_g is the cheapest so far.
                    if (test_g < neighbor_node->cost_g)
                    {
                        neighbor_node->cell = neighbor_cell; // cell coordinate was not initialized
                        neighbor_node->cost_g = test_g;
                        // h_cost = distance between neighbor_node and goal point // using manhattan distance in this case
                        neighbor_node->cost_h = abs(goal_cell.x - neighbor_cell.x) + abs(goal_cell.y - neighbor_cell.y); // FIXME 
                        neighbor_node->cost_f = neighbor_node->cost_g + neighbor_node->cost_h; // FIXME // f_cost ← h_cost + (neighbor_node's g cost)
                        neighbor_node->parent = expanded_node;

                        open_list_.queue(neighbor_node);
                    }
                }
            }

            // Clear open list
            open_list_.clear();

            return path();
        }

        /**
         * Find turning points on a path // helper function for cubic hermite splines smoother
         */
        std::vector<V2d> findTurningPoints(const std::vector<V2d> &path) {
            std::vector<V2d> turning_points;
            for (size_t i = 0; i < path.size(); ++i) {
                V2d coord = path[i]; // x_i
                // std::cout << "current coord: " << coord << std::endl;

                if (i == 0 || i == path.size() - 1) { // keep start and goal point
                    turning_points.push_back(coord); 
                    continue;
                }
                
                V2d x_prev = path[i - 1]; // x_(i-1)
                V2d x_next = path[i + 1]; // x_(i+1)
                V2d v_curr = coord - x_prev; // v_i
                V2d v_next = x_next - coord; // v_(i+1)
                double v_cross_product = v_curr.x * v_next.y - v_curr.y * v_next.x; // v_i x v_(i-1) // can use v_curr.cross(v_next) also
                if (abs(v_cross_product) > 1e-5) {  // if cross product !=0, the three points are not parallel, turning point found at x_i
                    turning_points.push_back(coord);
                }
            } 
            return turning_points;
        }

        /**
         * Calculate velocity direction at each turning point // helper function for cubic hermite splines smoother
         */
        V2d calcVelocityDirection(const V2d &point, const V2d &point_prev, const V2d &point_next) {
            // velocity direction at each turning point = average of unit directional vectors of both adjacent segments
            // ^ i think this means if at p2, velocity direction is average of line direction p1-p2 and line direction p2-p3
            V2d v1 = point - point_prev;
            V2d v2 = point_next - point;
            V2d v = (v1 / v1.norm() + v2 / v2.norm()) / 2;
            return v;
        }

        /**
         * Implement smoother using cubic hermite splines 
         */
        std::vector<V2d> cubic_hermite_spline_smoother(const std::vector<V2d> &path) {
            // find turning points along found path
            std::vector<V2d> turning_points = findTurningPoints(path);

            // for each segment on the new path, generate the cubic spline
            std::vector<V2d> smooth_path;
            double vel = params_.spline_vel; // use to determine curvature of spline
            for (size_t i = 0; i < turning_points.size() - 2; i++) {
                V2d p0 = turning_points[i]; // start point
                V2d p1 = turning_points[i + 1]; // end point

                // calculate time taken
                double dx = p1.x - p0.x, dy = p1.y - p0.y;
                double distance = sqrt(dx * dx + dy * dy); // can use (p0 - p1).norm() also
                double time = distance / vel; // time = distance / velocity
                
                // calculate velocity at each turning point
                V2d v0, v1;
                if (i == 0) { // first turning point
                    v0 = (p1 - p0).norm() * vel; // velocity of first point
                    v1 = calcVelocityDirection(p1, p0, turning_points[i + 2]) * vel;
                } else if (i == turning_points.size() - 2) { //last turning point
                    v0 = calcVelocityDirection(p0, turning_points[i - 1], p1) * vel;
                    v1 = (p1 - p0).norm() * vel; // velocity of last point
                } else {
                    v0 = calcVelocityDirection(p0, turning_points[i - 1], p1) * vel;
                    v1 = calcVelocityDirection(p1, p0, turning_points[i + 2]) * vel;
                }

                // calculate constants
                double t = time, t2 = t * t, t3 = t2 * t; // t^2, t^3
                double a0 = p0.x;
                double a1 = v0.x;
                double a2 = -3 / t2 * p0.x - 2 / t  * v0.x + 3 / t2 * p1.x - 1 / t  * v1.x;
                double a3 =  2 / t3 * p0.x + 1 / t2 * v0.x - 2 / t3 * p1.x + 1 / t2 * v1.x;
                double b0 = p0.y;
                double b1 = v0.y;
                double b2 = -3 / t2 * p0.y - 2 / t  * v0.y + 3 / t2 * p1.y - 1 / t  * v1.y;
                double b3 =  2 / t3 * p0.y + 1 / t2 * v0.y - 2 / t3 * p1.y + 1 / t2 * v1.y;
                
                // interpolate
                smooth_path.push_back(p0); // add start point to final smooth path
                double dt = 0.2; // TODO to tune or add as parameter to params
                for (double t = dt; t < time; t += dt) { 
                    double t2 = t * t, t3 = t2 * t; // t^2, t^3
                    double x_new = a0 + a1 * t + a2 * t2 + a3 * t3;
                    double y_new = b0 + b1 * t + b2 * t2 + b3 * t3;
                    V2d p_new = V2d(x_new, y_new);
                    smooth_path.push_back(p_new);
                }
                // dont need to add end point because it's new start point for next segment
            }

            smooth_path.push_back(turning_points.back()); // add last turning point to final smooth path
            return smooth_path;
        }

        /**
         * Implement smoother using Savitsky-Golay Moving Average 
         * assume points at regular intervals aka A* path
         * TODO: if Theta* neede interpolate so that points are regularly spaced
         */
        std::vector<V2d> savitsky_golay_smoother(const std::vector<V2d> &path, const int &half_window_size = 2, const int &poly_order = 3) {
            const int m = half_window_size, p = poly_order; // default cubic polynomial over 5 points
            const int row_size  = 2 * m + 1, col_size = p + 1;
            Eigen::MatrixXd J(row_size, col_size); // Vandermonde matrix, which is a (2m+1)×(p+1) matrix
            for (int r = 0; r < row_size; r++) {
                for (int c = 0; c < col_size; c++) {
                    J(r, c) = pow(-m + r, c);
                }
            }
            Eigen::MatrixXd a = (J.transpose() * J).inverse() * J.transpose(); // (J^T * J)^-1 * J^T
            Eigen::MatrixXd a_first_row = a.row(0);

            int n = path.size();
            std::vector<V2d> smooth_path;
            smooth_path.push_back(path[0]); // add start point to final smooth path

            for (int j = 1; j < n - 1; j++) { // iterate from second (1) point to second last (n-2) point
                V2d smoothed_point = V2d(0, 0);
                int a_idx = 0;
                for (int i = -m; i <= m; i++) {
                    int idx = j + i;
                    double a_val = a_first_row(a_idx);
                    if (idx < 0) { // point lies before the path, use x0 (first point)
                        smoothed_point += V2d(a_val* path[0].x, a_val * path[0].y);
                    } else if (idx >= n) { // point lies after path, use xn (last point)
                        smoothed_point += V2d(a_val* path[n - 1].x, a_val * path[n - 1].y);
                    } else { // for points with index m to n - m
                        smoothed_point += V2d(a_val* path[idx].x, a_val * path[idx].y);
                    }
                    a_idx++;
                }
                smooth_path.push_back(smoothed_point);
            }
            smooth_path.push_back(path[n - 1]); // add last point to final smooth path

            return smooth_path;
        }

        /**
         * Smooths the path into a more feasible trajectory
         */
        const std::vector<V2d> &smooth() // FIXME
        {
            // cubic hermite splines smoother
            std::vector<V2d> cubicHermite_smooth_path = cubic_hermite_spline_smoother(path_);

            // savitsky-golay moving average smoother
            std::vector<V2d> savitskyGolay_smooth_path = savitsky_golay_smoother(path_, 3, 3);

            // replace the path_ with the smooth_path
            path_ = cubicHermite_smooth_path;
            
            std::cout << "smoothed path: {"; 
            for (size_t i = 0; i < path_.size(); ++i) {
                  std::cout << path_[i] << "; ";
            }
            std::cout << "}" << std::endl;

            return path(); // returns path_
        }

        bool checkPointWithinInflation(const V2d &point) {
            V2 cell = inflation_layer_.worldToCell(point);
            long idx = inflation_layer_.cellToIdx(cell);
            int cost = inflation_layer_(idx);
            // std::cout << point << " with cost " << cost << std::endl;
            int LETHAL_COST = 5; // TODO: to tune and add to params?
            return cost >= LETHAL_COST;
        }

        /**
         * Updates the local copy of the cost map (inflation layer) to search the paths on.
         */
        void updateCostMap(const nav_msgs::msg::OccupancyGrid &msg) { inflation_layer_.copyFrom(msg); }

        /**
         * Returns the path.
         */
        const std::vector<V2d> &path() const { return path_; }

        /**
         * Returns the path as a nav_msgs::msg::Path message.
         */
        nav_msgs::msg::Path msg() const
        {
            nav_msgs::msg::Path msg_;
            msg_.header.frame_id = params_.frame_id;

            for (const V2d &coord : path())
            {
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = coord.x;
                pose.pose.position.y = coord.y;
                pose.pose.orientation.w = 1;
                msg_.poses.push_back(pose);
            }

            return msg_;
        }
    };

    /**
     * The Planner ROS Node that maintains subibers and publishers for the Planner class.
     */
    class ROSNodePlannerSmoother : public rclcpp::Node
    {
    private:
        PlannerSmootherParameters params_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;              // publisher
        rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr client_inflation_layer_; // client
        rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr service_plan_;         // service
        rclcpp::Service<ee4308_interfaces::srv::PathWithinInflation>::SharedPtr service_check_path_; // service
        std::shared_ptr<PlannerSmoother> planner_smoother_;                       // the planner and smoothing class tied to this node.
        rclcpp::CallbackGroup::SharedPtr cb_group_;                               // to allow all callbacks to simultaneously occur in the executor. Requires node to be added to MultiThreaderExecutor
        std::mutex mutex_;                                                        // required to prevent data races

    public:
        /**
         * Constructor for the Planner ROS Node.
         * @param name name of node.
         */
        ROSNodePlannerSmoother(
            const std::string &name = "planner_smoother")
            : Node(name)
        {
            // Initialize call back group
            cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

            // get all parameters from the param server.
            initParams();

            // Initialize topics
            initTopics();

            // Initialize Services
            initServices();

            // Initialize the planner
            planner_smoother_ = std::make_shared<PlannerSmoother>(params_);
        }

    private:
        /**
         * Initialize parameters from the parameter server
         */
        void initParams()
        {
            declare_parameter<std::string>("topics.plan", params_.topics.plan);
            get_parameter<std::string>("topics.plan", params_.topics.plan);
            RCLCPP_INFO_STREAM(get_logger(), "topics.plan: " << params_.topics.plan);

            declare_parameter<std::string>("services.get_inflation_layer", params_.services.get_inflation_layer);
            get_parameter<std::string>("services.get_inflation_layer", params_.services.get_inflation_layer);
            RCLCPP_INFO_STREAM(get_logger(), "services.get_inflation_layer: " << params_.services.get_inflation_layer);

            declare_parameter<std::string>("services.check_path_within_inflation", params_.services.check_path_within_inflation);
            get_parameter<std::string>("services.check_path_within_inflation", params_.services.check_path_within_inflation);
            RCLCPP_INFO_STREAM(get_logger(), "services.check_path_within_inflation: " << params_.services.check_path_within_inflation);

            declare_parameter<std::string>("services.get_plan", params_.services.get_plan);
            get_parameter<std::string>("services.get_plan", params_.services.get_plan);
            RCLCPP_INFO_STREAM(get_logger(), "services.get_plan: " << params_.services.get_plan);

            declare_parameter<std::string>("frame_id", params_.frame_id);
            get_parameter<std::string>("frame_id", params_.frame_id);
            RCLCPP_INFO_STREAM(get_logger(), "frame_id: " << params_.frame_id);

            declare_parameter<double>("spline_vel", params_.spline_vel);
            get_parameter<double>("spline_vel", params_.spline_vel);
            RCLCPP_INFO_STREAM(get_logger(), "spline_vel: " << params_.spline_vel);
        }

        /**
         * Initialize subscribers and wait until messages from all subscribed topics arrive.
         */
        void initTopics()
        {
            // Initialize Publishers
            pub_path_ = create_publisher<nav_msgs::msg::Path>(params_.topics.plan, 1);
        }

        /**
         * Initialize services and wait for them to respond
         */
        void initServices()
        {
            client_inflation_layer_ = create_client<nav_msgs::srv::GetMap>(
                params_.services.get_inflation_layer,
                rmw_qos_profile_services_default, cb_group_);
            service_plan_ = create_service<nav_msgs::srv::GetPlan>(
                params_.services.get_plan,
                std::bind(&ROSNodePlannerSmoother::servicePlan, this, std::placeholders::_1, std::placeholders::_2),
                rmw_qos_profile_services_default, cb_group_);
            service_check_path_ = create_service<ee4308_interfaces::srv::PathWithinInflation>(
                params_.services.check_path_within_inflation,
                std::bind(&ROSNodePlannerSmoother::serviceCheckPath, this, std::placeholders::_1, std::placeholders::_2),
                rmw_qos_profile_services_default, cb_group_);

            // wait for service to respond
            while (rclcpp::ok() == true &&
                   client_inflation_layer_->wait_for_service(std::chrono::duration<int, std::milli>(200)) == false) // wait for 200ms
            {
                // RCLCPP_INFO_STREAM(get_logger(), "Waiting for service servers...");
            }

            RCLCPP_INFO_STREAM(get_logger(), "Finished services.");
        }

        /**
         * Requests a map from mapper. Returns true if a map is received and copied.
         */
        bool requestInflationLayer()
        {
            // Wait for the service to respond.

            // Send the request.
            auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
            auto result = client_inflation_layer_->async_send_request(request);

            // wait for timeout
            while (rclcpp::ok() == true && result.future.valid() == false)
            { // wait until it responds.
            }

            if (rclcpp::ok() == false)
                return false;

            nav_msgs::msg::OccupancyGrid msg = result.get()->map;

            // update the internal map in the planner.
            planner_smoother_->updateCostMap(msg);

            return true;
        }

        /**
         * The service callback to respond with the path
         */
        void servicePlan(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
                         std::shared_ptr<nav_msgs::srv::GetPlan::Response> response)
        {
            // wait until a lock is acquired. Released when this function returns.
            const std::lock_guard<std::mutex> lock(mutex_);

            // request a map. If shutdown or failed, return the default plan (nothing).
            if (requestInflationLayer() == false)
            {
                std::cout << "Inflation layer cannot be requested. No path returned." << std::endl;
                return;
            }

            // retrieve the request's start and goal coordinates
            V2d start_coord = {request->start.pose.position.x, request->start.pose.position.y};
            V2d goal_coord = {request->goal.pose.position.x, request->goal.pose.position.y};

            std::cout << "======================================" << std::endl;

            // find the shortest path and store the path within the class.
            std::cout << "Running Planner..." << std::endl;
            planner_smoother_->run(start_coord, goal_coord);
            std::cout << "Run complete." << std::endl;

            // smooths the path / generate smooth trajectory
            planner_smoother_->smooth();

            // get and stamp the message
            nav_msgs::msg::Path msg = planner_smoother_->msg();
            msg.header.stamp = now();

            // publish the path
            pub_path_->publish(msg);

            // respond with the path
            response->plan = msg;
        }

        void serviceCheckPath(const std::shared_ptr<ee4308_interfaces::srv::PathWithinInflation::Request> request,
                                 std::shared_ptr<ee4308_interfaces::srv::PathWithinInflation::Response> response) {
            
            auto path = request->path.poses;
            // request a map. If shutdown or failed, return the default plan (nothing).
            if (requestInflationLayer() == false) {
                std::cout << "Inflation layer cannot be requested. Check failed." << std::endl;
                return;
            }
            std::vector<V2d> path_vector;
            for(std::vector<geometry_msgs::msg::PoseStamped>::const_iterator it = path.begin(); it != path.end(); ++it) {
                V2d point = V2d(it->pose.position.x, it->pose.position.y);
                if (planner_smoother_->checkPointWithinInflation(point)) {
                    response->path_within_inflation = true;
                    return;
                }
            }
            response->path_within_inflation = false;
        }
    };
}