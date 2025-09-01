#include <iostream>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <cmath>
#include <casadi/casadi.hpp>
#include <vector>

using namespace casadi;

int main()
{
  MX x = MX::sym("x"); 
  double k_star = 0;
  double y_star = 0 ;
  double x_star = 0 ;

  int num_points = 20;
  std::vector<double> x_values(num_points);
  double step = 1.0 / (num_points - 1);
  for (int j = 0; j < num_points; ++j) {
        x_values[j] = x_star + j * step;
      }

  std::vector<double> Y_STAR;
  std::vector<double> X_STAR;

 
  std::vector<double> ct_coeff = {0,0,0,0}; 
  MX global_path =  ct_coeff[0] * pow(x, 3) + ct_coeff[1] * pow(x, 2) + ct_coeff[2] * x + ct_coeff[3];
    // ob_info
  std::vector<double> obs = {1.5, 0.5, 0.5, 0.5, 0.0};

  //假设: 2m,的全局规划，dlx为1m;
  double N = 2;
    // Parameters
  MX p = MX::sym("p", 8); 
    //0. N ： 需要几个三次函数
    //1. dlx： 每个三次函数的距离
    //2. range_radar = 4
    //3. car_width = 1
    //4. car_length = 1
    //5. safe_dis = 0.5
    //6. y_up    = 3
    //7. y_down  = -3

  std::vector<casadi::Matrix<double>> U;

  for (size_t i =0 ; i< N; ++i){

    MX u = MX::sym("u", 4);

    //path defin
    MX y_inter = u(0) * pow(x, 3) + u(1) * pow(x, 2) + u(2) * x + u(3);
    MX k = 3 * u(0) * pow(x, 2) + 2 * u(1) * x + u(2); 
    MX k2 = 6 * u(0) * x + 2 * u(1);
    MX f_up_y = y_inter + (p(3) / 2) * (1 / sqrt(pow(k, 2) + 1));
    MX f_down_y = y_inter - (p(3) / 2) * (1 / sqrt(pow(k, 2) + 1));
    MX f_up_x = x - ((p(3) / 2) * k) / sqrt(pow(k, 2) + 1);
    MX f_down_x = x + ((p(3) / 2) * k) / sqrt(pow(k, 2) + 1);
    Function y_fn = Function("y_fn", {x, u}, {y_inter});
    Function k_fn = Function("k_fn", {x, u}, {k});
    Function k2_fn = Function("k2_fn", {x, u}, {k2});
    Function f_up_y_fn = Function("f_up_y_fn", {x, u}, {f_up_y});
    Function f_down_y_fn = Function("f_down_y_fn", {x, u}, {f_down_y});
    Function f_up_x_fn = Function("f_up_x_fn", {x, u}, {f_up_x});
    Function f_down_x_fn = Function("f_down_x_fn", {x, u}, {f_down_x});


    /*// k_star
    if (i==0){
      x_star = 0;
      k_star = 0;
      y_star = ct_coeff[0] * pow(x_star, 3) + ct_coeff[1] * pow(x_star, 2) + ct_coeff[2] * x_star + ct_coeff[3];
      for (int j = 0; j < num_points; ++j) {
        x_values[j] = x_star + j * step;
      }
    }

    else{
      x_star += i;
      auto k_val = 3 * U[i-1](0) * pow(x_star, 2) + 2 *  U[i-1](1) * x_star +  U[i-1](2);
      k_star = k_val.scalar();
      auto y_val = U[i-1](0) * pow(x_star, 3) + U[i-1](1) * pow(x_star, 2) + U[i-1](2) * x_star + U[i-1](3);
      y_star = y_val.scalar();
      for (int j = 0; j < num_points; ++j) {
        x_values[j] = x_star + j * step;
      }
    }*/
    //integrator
   std::vector<MX> coeff_diff(4);
   coeff_diff[0] = u(0) - ct_coeff[0];
   coeff_diff[1] = u(1) - ct_coeff[1];
   coeff_diff[2] = u(2) - ct_coeff[2];
   coeff_diff[3] = u(3) - ct_coeff[3];
   MX inter_diff_sq = (pow(coeff_diff[0],2)/7)*pow(x,7)+
                      ((2*coeff_diff[0]*coeff_diff[1]/6)*pow(x,6))+
                      (((2*coeff_diff[0]*coeff_diff[2]+pow(coeff_diff[1],2))/5)*pow(x,5))+
                      (((2*coeff_diff[0]*coeff_diff[3]+2*coeff_diff[1]*coeff_diff[2])/4)*pow(x,4))+
                      (((2*coeff_diff[1]*coeff_diff[3]+pow(coeff_diff[2],2))/3)*pow(x,3))+
                      (((2*coeff_diff[2]*coeff_diff[3])/2)*pow(x,2))+
                      (pow(coeff_diff[3],2)*x);
    Function intergral = Function("intergral",{x,u},{inter_diff_sq});
    MX term = intergral({x_star+1,u})[0]-intergral({x_star,u})[0];
    MX f = term;
    /*double global_path_star_val = ct_coeff[0] * pow(x_star, 3) + ct_coeff[1] * pow(x_star, 2) + ct_coeff[2] * x_star + ct_coeff[3];
    double global_path_end_val = ct_coeff[0] * pow(x_star+i, 3) + ct_coeff[1] * pow(x_star+i, 2) + ct_coeff[2] * x_star+i + ct_coeff[3];
    MX integral = 0.5*(sq(y_fn({x_star,u})[0] - global_path_star_val) + sq(y_fn({x_star+1,u})[0] - global_path_end_val));
    for (double x_val : x_values){
      integral  += (y_fn({x_val,u})[0] - (ct_coeff[0] * pow(x_val, 3) + ct_coeff[1] * pow(x_val, 2) + ct_coeff[2] * x_val + ct_coeff[3]));
    }
    MX f = integral;*/



    //Constraint: 1. up and down
    //std::vector x_ob = {obs[0],obs[0]-obs[2]/2,obs[0]+obs[3]/2};
    std::vector<MX> g_1_vec;
    std::vector<MX> g_2_vec;
    std::vector<MX> g_3_vec;
    std::vector<MX> g_4_vec;
    for (double x_val : x_values){
        MX g1 = -f_up_y_fn({x_val, u})[0] + p(6) ;
        MX g2 = f_down_y_fn({x_val, u})[0] - p(7);
        MX obs_dis = pow((x_val-obs[0])*cos(obs[4]) + (y_fn({x_val,u})[0]-obs[1])*sin(obs[4]),2)/pow(obs[2],2) + 
                     pow((x_val-obs[0])*sin(obs[4]) - (y_fn({x_val,u})[0]-obs[1])*cos(obs[4]),2)/pow(obs[3],2);
        //MX obs_dis = sqrt(pow(x_val-obs[0],2) + pow(y_fn({x_val,u})[0]-obs[1],2));

        //MX obs_dis = pow((x_val-obs[0])*cos(obs[4]) + (f_down_y_fn({x_val,u})[0]-obs[1])*sin(obs[4]),2)/pow(obs[2],2) + 
         //            pow((x_val-obs[0])*sin(obs[4]) - (f_down_y_fn({x_val,u})[0]-obs[1])*cos(obs[4]),2)/pow(obs[3],2);
        MX g3 = obs_dis-1;

        g_1_vec.push_back(g1);
        g_2_vec.push_back(g2);
        g_3_vec.push_back(g3);
    }

    MX g4_y = y_fn({x_star, u})[0] - y_star;
    MX g4_k = k_fn({x_star, u})[0] - k_star;
    g_4_vec.push_back(g4_y);  
    g_4_vec.push_back(g4_k);
    /*for (double x_val : x_ob){
        MX obs_dis = pow((x_val-obs[0])*cos(obs[4]) + (y_fn({x_val,u})[0]-obs[1])*sin(obs[4]),2)/pow(obs[2],2) + pow((x_val-obs[0])*sin(obs[4]) - (y_fn({x_val,u})[0]-obs[1])*cos(obs[4]),2)/pow(obs[3],2);
        MX g3 = obs_dis-1-p(5);
        g_3_vec.push_back(g3);
    }*/
    MX g_1 = vertcat(g_1_vec);
    MX g_2 = vertcat(g_2_vec);
    MX g_3 = vertcat(g_3_vec);
    MX g_4 = vertcat(g_4_vec);

    MX g = vertcat(g_1, g_2, g_3,g_4);


    // Initial guess and bounds for the optimization variables
    std::vector<double> u0  = {0,0,0,0};
    std::vector<double> lbu = {-inf, -inf, -inf,-inf};
    std::vector<double> ubu = { inf,  inf,  inf,inf};
    // Nonlinear bounds
    size_t num_constraints = g_1_vec.size() + g_2_vec.size() + g_3_vec.size();
    std::vector<double> lbg(num_constraints+2, 0.0); 
    std::vector<double> ubg(num_constraints, inf); 
    ubg.push_back(0.0); 
    ubg.push_back(0.0); 

    // Original parameter values
    std::vector<double> p0  = {2, 1.00, 4.00, 1.00, 1.00, 0.5, 3.00, -3.00};
    MXDict nlp = {{"x", u}, {"p", p}, {"f", f}, {"g", g}};
    Function solver = nlpsol("solver", "ipopt", nlp);
    std::map<std::string, DM> arg, res;

      // Solve the NLP
    arg["lbx"] = lbu;
    arg["ubx"] = ubu;
    arg["lbg"] = lbg;
    arg["ubg"] = ubg;
    arg["x0"] = u0;
    arg["p"] = p0;
    res = solver(arg);
    U.push_back(res.at("x"));
    Y_STAR.push_back(y_star);
    X_STAR.push_back(x_star);

    //up_date
    //x_values.clear();
    x_star = i+1;
    auto y_val = U[i](0) * pow(x_star, 3) + U[i](1) * pow(x_star, 2) + U[i](2) * x_star + U[i](3);
    y_star = y_val.scalar();
    auto k_val = 3 * U[i](0) * pow(x_star, 2) + 2 *  U[i](1) * x_star +  U[i](2);
    k_star = k_val.scalar();
    for (int j = 0; j < num_points; ++j) {
        x_values[j] = x_star + j * step;
    }
  
  
    std::cout << std::setw(30) << "Dual solution (u): " << U << std::endl;
    std::cout << std::setw(30) << "Y_STAR " <<  Y_STAR<< std::endl;
    std::cout << std::setw(30) << "X_STAR " <<  X_STAR<< std::endl;
    //std::cout << std::setw(30) << "g_3 " <<  g_3_vec << std::endl;

  }

}