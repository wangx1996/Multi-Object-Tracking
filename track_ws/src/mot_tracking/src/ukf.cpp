/*
 * author: wx
 * date: 2020.12.08
 * reference:https://github.com/zhujun98/sensor-fusion
 */

#include "ukf.h"

float Polar_angle_cal(float x, float y) {
	float PI = 3.1415926;
	float temp_tangle = 0;
	if (x == 0 && y == 0) {
		temp_tangle = 0;
	} else if (y >= 0) {
		temp_tangle = (float) atan2(y, x);
	} else if (y <= 0) {
		temp_tangle = (float) atan2(y, x) + 2 * PI;
	}
	return temp_tangle;
}

/*float normalangle(float angle, float v){
	std::cout<<"noramel########### "<<angle<<std::endl;
	if(angle>2*M_PI || angle<0){
		float vx = v * std::cos(angle);
		float vy = v * std::sin(angle);
		angle = Polar_angle_cal(vx,vy);
	}
	std::cout<<"noramel###########END "<<angle<<std::endl;
	return angle;
}*/
float normalangle(float angle)
{
  const auto times = std::round(std::fabs(angle / (2. * M_PI)));  // for the case when angle is very very large

  if (angle > M_PI)
  {
    angle -= times * 2.0 * M_PI;
  }

  if (angle < -M_PI)
  {
    angle += times * 2.0 * M_PI;
  }
  return angle;
}


//state: position_x, position_y,velocity, yaw, yaw_rate
void UKF::Initialization(Eigen::VectorXd& X, Eigen::MatrixXd& P, float time){

	//std::cout<<"######################## UKFinitialize #######################"<<std::endl;
	x_ = X;
	P_ = P;

	pretime = time;

	lamda_ = 3 - n_x_;

	//权重
	pre_weight = Eigen::VectorXd(2 * n_x_ + 1);
	mea_weight = Eigen::VectorXd(2 * n_x_ + 1);
	pre_weight(0) = (lamda_/(lamda_+n_x_));
	mea_weight(0) = (lamda_/(lamda_+n_x_));
	float w = 0.5/(lamda_+n_x_);
	for(int i =1; i<2*n_x_+1; ++i){
		pre_weight(i) = (w);
		mea_weight(i) = (w);
	}
	is_initalized_ = true;
}

bool UKF::Isinitalized(){
	return is_initalized_;
}


//生成sigma点
void UKF::MakeSigmaPoints(){
	//cholskey分解，或得协方差矩阵的根
	Eigen::VectorXd x_aug_ = Eigen::VectorXd(n_x_);
	x_aug_.head(n_x_) = x_; 


	Eigen::MatrixXd P_aug = Eigen::MatrixXd::Zero(n_x_, n_x_);;
	P_aug = P_;

	Eigen::MatrixXd L = P_aug.llt().matrixL();
  	sigma_points = Eigen::MatrixXd(n_x_, 2 * n_x_ + 1);
	//std::cout<<"MakeSigmaPoints\n"<<L<<std::endl;

	sigma_points.col(0) = x_aug_;
  	const float c = std::sqrt(lamda_ + n_x_);


	for(int i=0; i<n_x_; ++i){
		sigma_points.col(i+1) = x_aug_ + c*L.col(i);
		sigma_points.col(i+n_x_+1) = x_aug_ - c*L.col(i);
	}

	//std::cout<<"MakeSigmaPointsresult: "<<'\n'<<sigma_points<<std::endl;
}

//
void UKF::Prediction(float ntime){

	//std::cout<<"Prediction"<<'\n'<<std::endl;
	float deltat = ntime - pretime;

	pretime = ntime;

	//Prediction sigam points
	sigma_points_pre = Eigen::MatrixXd(n_x_, 2 * n_x_ + 1);

	//std::cout<<"################ USING MODEL ################## "<<model_<<std::endl;

	for(int i=0; i<2*n_x_+1; ++i){

		float px_k(0.0),py_k(0.0),velo_k(0.0),yaw_k(0.0),yawd_k(0.0), a_k(0.0);
		px_k = sigma_points(0,i);
		py_k = sigma_points(1,i);
		velo_k = sigma_points(2,i);
		yaw_k = sigma_points(3,i);
		if(n_x_ == 5)
			yawd_k = sigma_points(4,i);
		if(n_x_ == 6){
			yawd_k = sigma_points(4,i);		
			a_k = sigma_points(5,i);
		}


		if(model_ == 1){  //CV
			float px_pre,py_pre,velo_pre,yaw_pre,yawd_pre,a_pre;
      		px_pre = px_k + velo_k * deltat * cos(yaw_k);
      		py_pre = py_k + velo_k * deltat * sin(yaw_k);

			velo_pre = velo_k;		
			yaw_pre = yaw_k; 

			sigma_points_pre(0,i) = px_pre;
			sigma_points_pre(1,i) = py_pre;
			sigma_points_pre(2,i) = velo_pre;
			sigma_points_pre(3,i) = yaw_pre;//normalangle(yaw_pre);//,velo_pre);
			sigma_points_pre(4,i) = yawd_k;
			sigma_points_pre(5,i) = a_k;

		}else if(model_ == 2){ //CTRV
			float px_pre,py_pre,velo_pre,yaw_pre,yawd_pre,a_pre;
   	 		if (fabs(yawd_k) > 0.001) {
      				px_pre = px_k + velo_k / yawd_k * (std::sin(yaw_k + yawd_k * deltat) - std::sin(yaw_k));
      				py_pre = py_k + velo_k / yawd_k * (std::cos(yaw_k) - std::cos(yaw_k + yawd_k * deltat));
    			}	
    			else {
      				px_pre = px_k + velo_k * deltat * cos(yaw_k);
      				py_pre = py_k + velo_k * deltat * sin(yaw_k);
    			}
			velo_pre = velo_k;		
			yaw_pre = yaw_k + yawd_k*deltat;
			yawd_pre = yawd_k;
	
			sigma_points_pre(0,i) = px_pre;
			sigma_points_pre(1,i) = py_pre;
			sigma_points_pre(2,i) = velo_pre;
			sigma_points_pre(3,i) = yaw_pre;//normalangle(yaw_pre);//,velo_pre);
			sigma_points_pre(4,i) = yawd_pre;
			sigma_points_pre(5,i) = a_k;

		}else if(model_ == 3){ //CTRA
			float px_pre,py_pre,velo_pre,yaw_pre,yawd_pre,a_pre;
			if(fabs(yawd_k) > 0.001) {
      				px_pre = px_k + ((velo_k * yawd_k + a_k * yawd_k * deltat)*std::sin(yaw_k + yawd_k * deltat) + a_k * std::cos(yaw_k + yawd_k * deltat) - velo_k * yawd_k * std::sin(yaw_k) - a_k * std::cos(yaw_k))/(yawd_k * yawd_k);
      				py_pre = py_k + ((-velo_k * yawd_k - a_k * yawd_k * deltat)*std::cos(yaw_k + yawd_k * deltat) + a_k * std::sin(yaw_k + yawd_k * deltat) + velo_k * yawd_k * std::cos(yaw_k) - a_k * std::sin(yaw_k))/(yawd_k * yawd_k);
    			}	
    			else {
      				px_pre = px_k + (velo_k * deltat + 0.5 * a_k * deltat * deltat) * cos(yaw_k) ;
      				py_pre = py_k + (velo_k * deltat + 0.5 * a_k * deltat * deltat) * sin(yaw_k) ;
    			}

			velo_pre = velo_k + a_k * deltat;		
			yaw_pre = yaw_k + yawd_k*deltat;
			yawd_pre = yawd_k;
			a_pre = a_k;

			sigma_points_pre(0,i) = px_pre;
			sigma_points_pre(1,i) = py_pre;
			sigma_points_pre(2,i) = velo_pre;
			sigma_points_pre(3,i) = yaw_pre;//normalangle(yaw_pre);//,velo_pre);
			sigma_points_pre(4,i) = yawd_pre;
			sigma_points_pre(5,i) = a_pre;
			for(int k = 6; k<n_x_ ; ++k){
				sigma_points_pre(k,i) = 0;			
			}
		}

	}

	//计算预测均值
	x_.fill(0.0);
	for(int i=0; i<2*n_x_+1; ++i){
		x_ += pre_weight(i)*sigma_points_pre.col(i);
	}

	//std::cout<<"########## 计算预测均值x_ ##########"<<'\n'<<x_<<std::endl;
	//计算预测协方差
	try{
		P_.fill(0.0);
		for(int i=0; i<2*n_x_+1; ++i){
			Eigen::VectorXd x_diff = sigma_points_pre.col(i)-x_;
			x_diff(3) =  normalangle(x_diff(3));//,x_diff(2));
			P_ += pre_weight(i) * x_diff * x_diff.transpose();
		}
      		
		P_ += Q_;
		//std::cout<<"计算预测协方差P_ "<<'\n'<<P_<<std::endl;
	}
	catch(std::bad_alloc){
		std::cout<<"erro"<<std::endl;
		return;
	}
}


void UKF::PredictionZ(Eigen::VectorXd& X,Eigen::MatrixXd& P, float ntime){
	//std::cout<<"############################## UKFPrediction START ###############################\n"<<std::endl;
	x_ = X;
	P_ = P;

	//std::cout<<x_<<std::endl;

	MakeSigmaPoints();

	Prediction(ntime);
	
	Z_sigma_ = Eigen::MatrixXd(n_z_, 2 * n_x_ + 1);
	Z_sigma_.fill(0.0);
	//预测的量测点
  	for (int i = 0; i < 2 * n_x_ + 1; ++i) {
		for(int j =0 ; j< n_z_; ++j){	
			Z_sigma_(j,i) = sigma_points_pre(j, i);
		}
	}

  	z_pre_ = Eigen::VectorXd(n_z_);
	z_pre_.fill(0.0);


	//量测均值
	for (int i = 0; i < 2 * n_x_ + 1; ++i) {	
		z_pre_ += mea_weight(i) * Z_sigma_.col(i);
	}
	//std::cout<<"########### UKF量测simagee ###########\n"<<Z_sigma_<<"\n"<<std::endl;

	//std::cout<<"########### UKF量测均值 ########### "<<model_<<"\n"<<z_pre_<<"\n"<<std::endl;

	//计算量测协方差
	S_.fill(0.0);
	for(int i=0; i < 2*n_x_ + 1; ++i){
		Eigen::VectorXd z_diff = Z_sigma_.col(i) - z_pre_;
		S_ += (mea_weight(i) * z_diff * z_diff.transpose());
	}

	S_ += R_;
	//std::cout<<"########### UKF计算量测协方差S ###########\n"<<S_<<"\n ########### INVERSE ###########\n"<<S_.inverse()<<"\n########### R ###########\n"<<R_<< std::endl;

	//std::cout<<"############################## UKFPrediction END ###################################"<<std::endl;
}


void UKF::Update( std::vector<Eigen::VectorXd>& Z, const Eigen::VectorXd& beta, const float& last_beta){
	//std::cout<<"############################## UKFUPdate START #######################################"<<std::endl;
	//std::cout<<"Update"<<std::endl;
	//最后更新	
	Eigen::MatrixXd T = Eigen::MatrixXd(n_x_,n_z_);
	T.fill(0.0);
	for(int i=0; i<2*n_x_+1; ++i){
		Eigen::VectorXd x_diff = sigma_points_pre.col(i)-x_;
		//x_diff(3) =  normalangle(x_diff(3));//,x_diff(2));
		Eigen::VectorXd z_diff = Z_sigma_.col(i)-z_pre_;
		T += mea_weight(i) * x_diff * z_diff.transpose();
	}
	//std::cout<<"最后更新T\n"<<T<<std::endl;

	//卡曼增益
	Eigen::MatrixXd K = Eigen::MatrixXd(n_x_,n_z_);
	K = T * S_.inverse();
	//std::cout<<"卡曼增益K\n"<<K<<std::endl;

	//JPDAF update
	Zminus_.fill(0.0);
	int i=0;
	Eigen::VectorXd x_filter = Eigen::VectorXd(n_x_);
	x_filter.fill(0.0);
	for(const auto& det : Z)
	{
		Eigen::VectorXd a = x_ + K * (det - z_pre_);
	    x_filter = x_filter + beta(i) * a;
	    ++i;
	}
	x_filter = last_beta * x_ + x_filter;

	Eigen::MatrixXd P_temp = Eigen::MatrixXd(n_x_, n_x_);
	P_temp.fill(0.);
	for(int i = 0; i < Z.size() + 1; ++i)
	{
		Eigen::VectorXd a = Eigen::VectorXd(n_x_);
		a.setZero();
	    if(i == Z.size()){
	    	a = x_;
	    }
	    else{
	    	a = x_ + K * (Z.at(i) - z_pre_);
	    }

	    P_temp = P_temp + beta(i) * (a * a.transpose() - x_filter * x_filter.transpose());
	}

	x_ = x_filter;
	P_ -= (1 - last_beta) * K * S_ * K.transpose();
	P_ += P_temp;


	/*for(const auto& det:Z){
		Zminus_ += beta(i)*(det-z_pre_);
		i++;
	}

	x_ += K * Zminus_;
	x_(3) = normalangle(x_(3));//,x_(2));
	P_ -= (1-last_beta)* K * S_ * K.transpose();

	Eigen::MatrixXd z_temp(n_z_,n_z_);
	z_temp.fill(0.0);
	
	int j =0;
	for(const auto& det:Z){
		z_temp += beta(j)*(det-z_pre_)*(det-z_pre_).transpose();
		j++;
	}

	
	P_ += K*(z_temp - Zminus_*Zminus_.transpose()) * K.transpose();	*/


	//std::cout<<"\n ########### end ###########\n"<<x_<<'\n'<<std::endl;
	//std::cout<<"############################## UKFUPdate END #################################"<<std::endl;
}


void UKF::Update( Eigen::VectorXd& Z){
	//std::cout<<"############################## UKFUPdate NORMAL START #######################################"<<std::endl;
	//std::cout<<"Update"<<std::endl;
	//最后更新
	Eigen::MatrixXd T = Eigen::MatrixXd(n_x_,n_z_);
	T.fill(0.0);
	for(int i=0; i<2*n_x_+1; ++i){
		Eigen::VectorXd x_diff = sigma_points_pre.col(i)-x_;
		//x_diff(3) =  normalangle(x_diff(3));//,x_diff(2));
		Eigen::VectorXd z_diff = Z_sigma_.col(i)-z_pre_;
		T += mea_weight(i) * x_diff * z_diff.transpose();
	}
	//std::cout<<"最后更新T\n"<<T<<std::endl;

	//卡曼增益
	Eigen::MatrixXd K = Eigen::MatrixXd(n_x_,n_z_);
	K = T * S_.inverse();
	//std::cout<<"卡曼增益K\n"<<K<<std::endl;

	//JPDAF update
	Zminus_.fill(0.0);
	Zminus_ = (Z - z_pre_);

	x_ += K * Zminus_;
	x_(3) = normalangle(x_(3));//,x_(2));
	P_ -= K * S_ * K.transpose();

	Eigen::MatrixXd z_temp(n_z_,n_z_);
	z_temp.fill(0.0);

	//std::cout<<"\n ########### end ###########\n"<<x_<<'\n'<<std::endl;
	//std::cout<<"############################## UKFUPdate NORMAL END #################################"<<std::endl;
}

void UKF::Process(Eigen::VectorXd& X, std::vector<Eigen::VectorXd>& Z, const Eigen::VectorXd& beta,const float& last_beta, Eigen::MatrixXd& P, float time){
	//初始化 
	//std::cout<<"########## UKF Process ##########"<<std::endl;
	if(!Isinitalized()){
		Initialization(X,P,time);						
		return;
	}else{
		PredictionZ(X, P, time);
		Update(Z, beta, last_beta);
	}
	return;
}


Eigen::VectorXd UKF::Get_state(){
	return x_;
}

Eigen::MatrixXd UKF::Get_covariance(){
	return P_;
}

Eigen::VectorXd UKF::Get_Zminus(){
	return Zminus_;
}

Eigen::MatrixXd UKF::Get_S(){
	return S_;
}

Eigen::VectorXd UKF::Get_PredictionZ(){
	return z_pre_;
}
