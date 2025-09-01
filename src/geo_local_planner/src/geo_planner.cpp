#include "geo_local_planner/geo_planner.h" 

#include <base_local_planner/goal_functions.h>
#include <tf2/utils.h>
#include <ros/package.h>




namespace geo_local_planner
{
    // ====================================
    // planner class definition implementation.
    // ====================================
    geo_planner::geo_planner()
    {}

    bool geo_planner::config_geo_planner()
    {
        obstaclePresent = false;
    
        nlp_params.reserve(20);
        ns = 40;
        n_obs = 5;


        //params
        X_val = 3;
        x_star_val = 0;
        y_star_val = 0;
        k_star_val = 0;
        car_width_val = 1; 
        safe_dis_val = 0.1;
        global_coeff_val.clear();
        lt_coeff_val.clear();
        obs_val.clear();
        lbg.clear();
        ubg.clear();
        lbu.clear();
        ubu.clear();
        u0 = {0,0,0,0};

         // Set options
        Dict opts;
        opts["ipopt.tol"] = 1e-5;
        opts["ipopt.max_iter"] = 100;
        opts["ipopt.print_level"] = 5;
        opts["ipopt.sb"] = "yes";
        opts["ipopt.acceptable_tol"] = 1e-5;
        opts["ipopt.acceptable_iter"] = 0;
        //opts["ipopt.linear_solver"] = linear_solver;
        // opts["ipopt.hessian_approximation"] = "limited-memory";
        opts["ipopt.warm_start_init_point"] = "yes";
        opts["ipopt.warm_start_bound_push"] = 1e-5;
        opts["ipopt.warm_start_mult_bound_push"] = 1e-5;
        opts["print_time"] = 0;    

        // bounds for continuity 
        //for (int i = 0; i < 3 * ns + ns * n_obs; ++i)
        for (int i = 0; i < 3 * ns + ns * n_obs; ++i)
        {
            lbg.push_back(0.0);
            ubg.push_back(std::numeric_limits<double>::infinity());
        }
        lbg.insert(lbg.end(), {0.0});
        ubg.insert(ubg.end(), {0.0});
        


        //obs_val.assign(10,15);
        _obstacles_vec.reserve(3);
        _obstacles_vec.clear();

        //solver
        string package_name = ros::package::getPath("geo_local_planner");
        string nlp_solver_name = "geo_local_path_";  
        solver = nlpsol("solver", "ipopt", package_name + "/gen_c/" + nlp_solver_name + ".so", opts);

        //Obstacles
        std::vector<double> pattern = {10, 10, 0.01, 0.01, 0};
        for (int i = 0; i < 25; ++i) {
            obs_val.push_back(pattern[i % pattern.size()]);
        }

        return true;        
    }

    void geo_planner::caculate_geo_path(vector<geometry_msgs::PoseStamped>& robot_plan, std::vector<geometry_msgs::PoseStamped> &leftSidewalk_plan, std::vector<geometry_msgs::PoseStamped> &rightSidwalk_plan)
    {    
        nlp_params.clear();
        // Number of waypoints
        int N = robot_plan.size(); 
        int NL = leftSidewalk_plan.size();
        int NR = rightSidwalk_plan.size();
        Eigen::VectorXd x_veh(N);
        Eigen::VectorXd y_veh(N);
        //sub global_planner
        for(int i =0 ; i < N; i++)
        {
            x_veh[i] = robot_plan[i].pose.position.x;
            y_veh[i] = robot_plan[i].pose.position.y; 
        }
        auto coeffs_veh = polyfit(x_veh, y_veh, 3);   

        global_coeff_val = {coeffs_veh[3], coeffs_veh[2], coeffs_veh[1], coeffs_veh[0]};
        ROS_INFO("global_coeff_val: [%.6f, %.6f, %.6f, %.6f]", 
             global_coeff_val[0], global_coeff_val[1], global_coeff_val[2], global_coeff_val[3]);


        //side
        if (!leftSidewalk_plan.empty())
        {
            Eigen::VectorXd x_veh_left(NL);
            Eigen::VectorXd y_veh_left(NL);
            for(int i =0 ; i < NL; i++)
            {
                x_veh_left[i] = leftSidewalk_plan[i].pose.position.x;
                y_veh_left[i] = leftSidewalk_plan[i].pose.position.y; 
            }
            //auto coeffs_left = polyfit(x_veh_left, y_veh_left, 3); 
            //lt_coeff_val = {coeffs_left[3], coeffs_left[2], coeffs_left[1], coeffs_left[0]};
            auto coeffs_left = polyfit(x_veh_left, y_veh_left, 1); 
            lt_coeff_val = {0, 0, coeffs_left[1], coeffs_left[0]};
            ROS_INFO("lt_coeff_val: [%.6f, %.6f, %.6f, %.6f]", 
            lt_coeff_val[0], lt_coeff_val[1], lt_coeff_val[2], lt_coeff_val[3]);
        }
        else
        {
            lt_coeff_val = {0.0, 0.0, 0, 3}; 
        }
         
        if (!rightSidwalk_plan.empty())
        {
            Eigen::VectorXd x_veh_right(NR);
            Eigen::VectorXd y_veh_right(NR);    
            for(int i =0 ; i < NR; i++)
            {
                x_veh_right[i] = rightSidwalk_plan[i].pose.position.x;
                y_veh_right[i] = rightSidwalk_plan[i].pose.position.y; 
            }    
            //auto coeffs_right = polyfit(x_veh_right, y_veh_right, 3); 
            //rt_coeff_val = {coeffs_right[3], coeffs_right[2], coeffs_right[1], coeffs_right[0]};    
            auto coeffs_right = polyfit(x_veh_right, y_veh_right, 1); 
            rt_coeff_val = {0, 0, coeffs_right[1], coeffs_right[0]};
            ROS_INFO("rt_coeff_val: [%.6f, %.6f, %.6f, %.6f]", 
            rt_coeff_val[0], rt_coeff_val[1], rt_coeff_val[2], rt_coeff_val[3]);
        }
        else
        {
            rt_coeff_val = {0.0, 0.0, 0, -3};
        }

        //ob_val 处理
        int obs_cnt = 0;
        ROS_INFO("_obstacles_vec: %zu", _obstacles_vec.size());
        for (auto obs_it = _obstacles_vec.begin(); obs_it != _obstacles_vec.end(); obs_it++)
        {
            if (obs_cnt>=5) break;
            obs_val[obs_cnt*5+0] = obs_it->X;
            obs_val[obs_cnt*5+1] = obs_it->Y;
            obs_val[obs_cnt*5+2] = obs_it->R1; 
            obs_val[obs_cnt*5+3] = obs_it->R2;
            obs_val[obs_cnt*5+4] = obs_it->Theta;
            obs_cnt++;
        }
        for (auto i = obs_cnt; i <5 ; ++i)
        {
            obs_val[i*5+0] = 10;
            obs_val[i*5+1] = 10;
            obs_val[i*5+2] = 0.01; 
            obs_val[i*5+3] = 0.01;
            obs_val[i*5+4] = 0;
        }
        for (int i = 0; i < obs_val.size()/5; ++i) {
        ROS_INFO("Obstacle %d: X=%.2f, Y=%.2f, R1=%.2f, R2=%.2f, Theta=%.2f",
                 i + 1,
                 obs_val[i * 5 + 0],
                 obs_val[i * 5 + 1],
                 obs_val[i * 5 + 2],
                 obs_val[i * 5 + 3],
                 obs_val[i * 5 + 4]);
        }
         // TODO: set_nlp_params
                 /*params = vertcat(X_val, 
                        car_width_val, 
                        global_coeff_val, 
                        lt_coeff_val, 
                        rt_coeff_val, 
                        obs_val, 
                        safe_dis_val, 
                        x_star_val, 
                        y_star_val, 
                        k_star_val)*/

        nlp_params.insert(nlp_params.end(), X_val);
        nlp_params.push_back(car_width_val);
        nlp_params.insert(nlp_params.end(), global_coeff_val.begin(), global_coeff_val.end());
        nlp_params.insert(nlp_params.end(), lt_coeff_val.begin(), lt_coeff_val.end());
        nlp_params.insert(nlp_params.end(), rt_coeff_val.begin(), rt_coeff_val.end());
        nlp_params.insert(nlp_params.end(), obs_val.begin(), obs_val.end());
        nlp_params.push_back(safe_dis_val);
        nlp_params.push_back(x_star_val);
        nlp_params.push_back(y_star_val);
        nlp_params.push_back(k_star_val);



        lbu = {-std::numeric_limits<double>::infinity(), 
               -std::numeric_limits<double>::infinity(),
               0,
               0};

        ubu = {std::numeric_limits<double>::infinity(), 
               std::numeric_limits<double>::infinity(),
               0,
               0};

        /*lbu = {-5, 
               -std::numeric_limits<double>::infinity(),
               -std::numeric_limits<double>::infinity(),
               0};

        ubu = {5, 
               std::numeric_limits<double>::infinity(),
               std::numeric_limits<double>::infinity(),
               0};*/



        map<string, DM> arg, res;

        arg["lbx"] = lbu;
        arg["ubx"] = ubu;
        arg["lbg"] = lbg;
        arg["ubg"] = ubg;
        arg["x0"] = u0;
        arg["p"] = nlp_params;

        res = solver(arg);
        vector<double> val(res.at("x"));
        u_opt = val;

        //clear obstacles
        _obstacles_vec.clear();
    }

    /*void geo_planner::geo_path_visualize(std::vector<geometry_msgs::PoseStamped> &geo_plan)
    {
        geometry_msgs::PoseStamped geo_pose;
        tf2::Quaternion q;
        //ns= 20, X_val, dx = X_val/ns, x_star_val=0
        for(size_t i =0 ; i < 10; i++)
        {   double x_pose = x_star_val + i*(X_val/10);
            double y_pose = u_opt[0] * std::pow(x_pose, 3) + u_opt[1] * std::pow(x_pose, 2) + u_opt[2] * x_pose + u_opt[3];
            double slope_pose = 3 * u_opt[0] * std::pow(x_pose, 2) + 2 * u_opt[1] * x_pose + u_opt[2];
            double theta = std::atan(slope_pose);

            geo_pose.header.frame_id = "base_link";
            geo_pose.pose.position.x = x_pose;
            geo_pose.pose.position.y = y_pose;
            q.setRPY( 0, 0, theta);
            tf2::convert(q, geo_pose.pose.orientation);
            geo_plan.push_back(geo_pose);

            ROS_DEBUG("geo_plann---x:%.3f,y:%.3f,theta:%.3f",
                x_pose,y_pose,
                theta);
        }

    }*/

    void geo_planner::geo_path_visualize(std::vector<geometry_msgs::PoseStamped> &geo_plan, std::vector<geometry_msgs::PoseStamped> &polyfit_plan)
    {
        geometry_msgs::PoseStamped geo_pose;
        geometry_msgs::PoseStamped polyfit_pose;
        tf2::Quaternion q;
        //ns= 20, X_val, dx = X_val/ns, x_star_val=0
        for(size_t i =0 ; i < 100; i++)
        {   double x_pose = x_star_val + i*(X_val/100);
            double y_pose = u_opt[0] * std::pow(x_pose, 3) + u_opt[1] * std::pow(x_pose, 2) + u_opt[2] * x_pose + u_opt[3];
            double slope_pose = 3 * u_opt[0] * std::pow(x_pose, 2) + 2 * u_opt[1] * x_pose + u_opt[2];
            double theta = std::atan(slope_pose);

            double x_pose_global = x_star_val + i*(X_val/100);
            double y_pose_global = global_coeff_val[0] * std::pow(x_pose_global, 3) + global_coeff_val[1] * std::pow(x_pose_global, 2) + global_coeff_val[2] * x_pose_global + global_coeff_val[3];
            double slope_pose_global = 3 * global_coeff_val[0] * std::pow(x_pose_global, 2) + 2 * global_coeff_val[1] * x_pose_global + global_coeff_val[2];
            double theta_global = std::atan(slope_pose_global);

            geo_pose.header.frame_id = "base_link";
            geo_pose.pose.position.x = x_pose;
            geo_pose.pose.position.y = y_pose;
            q.setRPY( 0, 0, theta);
            tf2::convert(q, geo_pose.pose.orientation);
            geo_plan.push_back(geo_pose);

            polyfit_pose.header.frame_id = "base_link";
            polyfit_pose.pose.position.x = x_pose_global;
            polyfit_pose.pose.position.y = y_pose_global;
            q.setRPY( 0, 0, theta_global);
            tf2::convert(q, polyfit_pose.pose.orientation);
            polyfit_plan.push_back(polyfit_pose);

            ROS_DEBUG("geo_plann---x:%.3f,y:%.3f,theta:%.3f",
                x_pose,y_pose,
                theta);
        }

    }

    void geo_planner::insertSegmentObs(b_segment seg, double inflation_rate)
    {
        OBSTACLES seg_obs;
        double p1_x, p1_y, p2_x, p2_y;

        p1_x = bg::get<0, 0>(seg); p1_y = bg::get<0, 1>(seg); 
        p2_x = bg::get<1, 0>(seg); p2_y = bg::get<1, 1>(seg);

        seg_obs.X = (p1_x + p2_x)/2.0; // <0, 0> p1.x, <1, 0> p2.x
        seg_obs.Y = (p1_y + p2_y)/2.0; // <0, 1> p1.y, <1, 1> p2.y
        seg_obs.R1 = sqrt(pow(p2_x - p1_x, 2) + pow(p2_y - p1_y, 2));
        seg_obs.R2 = inflation_rate * seg_obs.R1;
        seg_obs.Theta = atan2(p2_y - p1_y,p2_x - p1_x);

        _obstacles_vec.push_back(seg_obs);
    }

    void geo_planner::insertCircleObs(circle circ)
    {
        OBSTACLES circ_obs;
        b_point circle_center = circ.center;
        double radius = circ.radius;

        circ_obs.X = bg::get<0>(circle_center); // <0, 0> p1.x, <1, 0> p2.x
        circ_obs.Y = bg::get<1>(circle_center); // <0, 1> p1.y, <1, 1> p2.y
        circ_obs.R1 = radius;
        circ_obs.R2 = radius;
        circ_obs.Theta = 0.0;
        
        _obstacles_vec.push_back(circ_obs);
    }

    bool geo_planner::checkObstacleFootprintIntersection(const std::vector<geometry_msgs::PoseStamped> &plan, 
                                                         const std::vector<geometry_msgs::Point> &_footprint_spec, const std::vector<b_segment> &seg_obs_, const std::vector<circle> &circ_obs_)
    {
        bool obstaclePresent = false;
        if (!plan.empty())
        {
            std::vector<b_segment> seg_obs = seg_obs_;
            std::vector<circle> circ_obs = circ_obs_;
            for (auto &t_pose : plan)
            {
                tf2::Quaternion tf_quat; //四元数，提供数学运算
                tf2::fromMsg(t_pose.pose.orientation, tf_quat); //Assume quat_msg is a quaternion ros msg

                tf2::Matrix3x3 m(tf_quat); //m 是3*3矩阵，提取tf_quat
                double t_roll, t_pitch, t_yaw;
                m.getRPY(t_roll, t_pitch, t_yaw); // getRPY from tf2::Matrix3x3
                // transform footprint from local to global_frame
                geometry_msgs::PolygonStamped p_footprint;
                costmap_2d::transformFootprint(t_pose.pose.position.x,t_pose.pose.position.y,t_yaw,_footprint_spec,p_footprint);                 
                // transformed_plan_footprints.push_back(p_footprint);

                b_polygon transformed_poly;
                for (auto &p : costmap_2d::toPointVector(p_footprint.polygon)) //costmap_2d::toPointVector : 用来将 ROS 消息格式的 geometry_msgs::Polygon 转换为 std::vector<geometry_msgs::Point>
                    bg::append(transformed_poly, b_point(p.x,p.y)); 
                    //bg::append是 boost::geometry 的函数，用于向多边形对象添加点
                    //b_point: geometry_msgs::Point 转换为 b_point 类型
                    /*从 geometry_msgs::Polygon 转换为 std::vector<geometry_msgs::Point>

                        使用 costmap_2d::toPointVector 提取多边形顶点信息，便于处理每个点。
                        从 geometry_msgs::Point 转换为 b_point

                        将每个 ROS 点（geometry_msgs::Point）转换为 boost::geometry 的点（b_point），实现格式的匹配。
                        构建 boost::geometry 多边形

                        使用 bg::append 将多个 b_point 点依次加入到 b_polygon 中，构造出完整的多边形。

                        然后才可以用bg库里的函数运算*/
                for (auto seg_it = seg_obs.begin(); seg_it != seg_obs.end();)
                {
                    bool isSegIntersect = bg::intersects(*seg_it, transformed_poly);
                    // Boost.Geometry 提供的一个函数，用来检查两个几何对象是否相交
                    
                    if(isSegIntersect)
                    {
                        ROS_INFO("seg obstacle intersected");

                        insertSegmentObs(*seg_it, _seg_inflation_rate);
                        // path_length = max(abs(NMPC_planner::_nmpc->obs_x),2.0);
                        ROS_DEBUG("segment intersected at path x: %.4f, path y:%.4f, seg_point x: %.4f y %.4f",t_pose.pose.position.x, t_pose.pose.position.y,
                        bg::get<0, 0>(*seg_it),bg::get<0, 1>(*seg_it));
                        seg_it = seg_obs.erase(seg_it);    
                        
                        ROS_INFO("seg obstacle inserted");
                        obstaclePresent = true;
                    }
                    else ++seg_it;
                }

                //ROS_INFO("circle_obs: %zu", circ_obs.size());
                for (auto circ_it = circ_obs.begin(); circ_it != circ_obs.end();)
                {

                    b_point circle_center = circ_it->center;
                    b_point pred_pose(t_pose.pose.position.x,t_pose.pose.position.y);
                    double obs_dist = bg::distance(circle_center,pred_pose);
                    double radius = circ_it->radius;
                    double isCircIntersect = false;
                    if (obs_dist<radius + 2) isCircIntersect = true;

                    if (isCircIntersect)
                    {
                        ROS_INFO("circle obstacle intersected");
                        insertCircleObs(*circ_it);
                        // path_length = max(abs(NMPC_planner::_nmpc->obs_x),2.0);
                        ROS_DEBUG("circle intersected at path x: %.4f, path y:%.4f, circle_point x: %.4f y: %.4f",t_pose.pose.position.x, t_pose.pose.position.y, 
                        bg::get<0>(circ_it->center),bg::get<1>(circ_it->center));
                        // circ_obs.erase(circ_it);
                        ROS_INFO("circle obstacle inserted");
                        obstaclePresent = true;
                        circ_it = circ_obs.erase(circ_it);
                    }
                    else ++circ_it;
                    // }
                }
         
            }
        }        
        return obstaclePresent;
    }

    Eigen::VectorXd geo_planner::polyfit(const Eigen::VectorXd &xvals, const Eigen::VectorXd &yvals, int order)
    {
    // 参数检查
        assert(xvals.size() == yvals.size());

        //assert(order >= 1 && order <= xvals.size() - 1);

        // 限制点数为 max_points 或实际点数
        //int n = std::min(static_cast<int>(xvals.size()), 40);

        // 取前n个点
        //Eigen::VectorXd xvals_limited = xvals.head(n);
        //Eigen::VectorXd yvals_limited = yvals.head(n);

        int n = xvals.size();
        Eigen::MatrixXd A(n, order + 1);
        for (int i = 0; i < xvals.size(); i++)
        A(i, 0) = 1.0;

        for (int j = 0; j < xvals.size(); j++) 
        {
            for (int i = 0; i < order; i++) 
                A(j, i + 1) = A(j, i) * xvals(j);
        }
        auto Q = A.householderQr();
        auto coeffs = Q.solve(yvals);

        return coeffs;
    }

}









