/*
 * author: wx
 * date: 2020.12.13
 * reference:
 */
#include "imm_ukf.h"

void IMM_UKF::IMM_Initialization(Eigen::VectorXd& Z, float time, float velo, float angle){
	//TODO initial x_merge_ interact_pro_
	isinitialized = true;
	
	filewrite.open("/home/wx/Desktop/tracking_code/trackernew/result.txt", std::ios::out|std::ios::app);

	Eigen::VectorXd x(n_x_);
	x.fill(0.0);
	for(int k=0; k<n_z_; ++k){
		x(k) = Z(k);
	}

	x(n_z_) = velo;
	x(n_z_+1) = angle;
	//初始化状态量
	for(int i=0; i<model_size; ++i){
		model_X_[i] = x;
	}
	//std::cout<<"############################## IMM initial ########################### "<<velo<<std::endl;

	for(int i=0; i<model_size; ++i){
		imm_ukf_[i].Initialization(model_X_[i],model_P_[i], time);
	}
}

//输入交互
void IMM_UKF::InputInteract(){
	//std::cout<<"########## IMM InputInteract ##########\n"<<model_pro_<<std::endl;
	if(std::isnan(model_pro_(0)))
		std::abort();
	if(model_pro_.sum() !=0)
		model_pro_ /model_pro_.sum();

	c_.fill(0.0);
	//the jth model
	for(int j=0; j<model_size; ++j){
		model_X_[j] = imm_ukf_[j].Get_state();//上一时刻的结果
		model_P_[j] = imm_ukf_[j].Get_covariance();
		for(int i=0; i<model_size; ++i){
			c_(j) += interact_pro_(i,j)*model_pro_(i); //i转移到j的概率
		}
	}
	//std::cout<<"########## IMM InputInteract ##########"<<std::endl;
	for(int j=0; j<model_size; ++j){
		X_hat_[j].fill(0.);
		P_hat_[j].fill(0.);
		for(int i=0; i<model_size; ++i){
			float u =  ((interact_pro_(i,j)*model_pro_(i))/c_(j));
			X_hat_[j] += u * model_X_[i]; //problem
		}
		for(int i=0; i<model_size; ++i){
			float u =  (interact_pro_(i,j)*model_pro_(i))/c_(j);		
			P_hat_[j] += (u * (model_P_[i] + (model_X_[i] - X_hat_[j])*(model_X_[i] - X_hat_[j]).transpose()));
		}
	}		
	
}	

//预测
void IMM_UKF::PredictionZmerge(float time){
	//std::cout<<"############################## IMM Prediction #############################"<<std::endl;
	InputInteract();

	std::vector<float> model_ita(model_size);

	S_merge_ = Eigen::MatrixXd(n_z_,n_z_);
	S_merge_.fill(0.0);

	Zpre_merge_ = Eigen::VectorXd(n_z_);
	Zpre_merge_.fill(0.0);
	//std::cout<<"############################## IMM Prediction1 #############################"<<std::endl;

	for(int i=0; i<model_size; ++i){
		imm_ukf_[i].PredictionZ(X_hat_[i], P_hat_[i], time);
		//TODO get zmerge
		Zpre_merge_ += (model_pro_(i) * imm_ukf_[i].Get_PredictionZ());
	}
	//std::cout<<"############################## IMM Prediction2 #############################"<<std::endl;

	for(int i=0; i<model_size; ++i){
		Eigen::MatrixXd S = imm_ukf_[i].Get_S();
		Eigen::VectorXd Zp = imm_ukf_[i].Get_PredictionZ();
		S_merge_ += ( model_pro_(i) * (S + (Zp - Zpre_merge_)*(Zp - Zpre_merge_).transpose()));
	}

	//std::cout<<"########## Zpre_merge_ ##########\n"<<Zpre_merge_<<"\n ########## Spre_merge_ ##########\n"<<S_merge_<<"\n"<<std::endl;

	//std::cout<<"#################### IMM Prediction END #############################"<<std::endl;
}

//模型以及概率更新		
void IMM_UKF::UpdateProbability(std::vector<Eigen::VectorXd>& Z, const Eigen::VectorXd& beta, const float& last_beta){
	//std::cout<<"#################### IMM UpdateProbability #############################"<<std::endl;

	std::vector<float> model_ita(model_size);

	for(int i=0; i<model_size; ++i){

		imm_ukf_[i].Update(Z, beta, last_beta);
	
		Eigen::VectorXd Zminus = imm_ukf_[i].Get_Zminus();
		Eigen::MatrixXd S = imm_ukf_[i].Get_S();

		float ita = 1/(sqrt(2*pi_)*sqrt(fabs(S.determinant())))*exp(-0.5* Zminus.transpose() * S.inverse() * Zminus);
		model_ita[i] = ita;
	}
	
	float c_temp = 0;	
	for(int i=0; i<model_size; ++i){
		c_temp += model_ita[i] * c_(i);
	}

	for(int i=0; i<model_size; ++i){
		model_pro_(i) = (model_ita[i]*c_(i))/c_temp;
		if(model_pro_(i)<1e-4) model_pro_(i) = 1e-4;
	}


	filewrite<<Z[0](0)<<" "<<Z[0](1)<<" ";
	MixProbability();

	//std::cout<<"########## Model probability ##########\n "<<model_pro_<<std::endl;
	//std::cout<<"############################## IMM UpdateProbability END #############################"<<std::endl;
}

//模型以及概率更新
void IMM_UKF::UpdateProbability(Eigen::VectorXd& Z){
	//std::cout<<"#################### IMM UpdateProbability #############################"<<std::endl;

	std::vector<float> model_ita(model_size);
	std::cout<<model_size<<std::endl;
	for(int i=0; i<model_size; ++i){

		imm_ukf_[i].Update(Z);

		Eigen::VectorXd Zminus = imm_ukf_[i].Get_Zminus();
		Eigen::MatrixXd S = imm_ukf_[i].Get_S();

		float ita = 1/(sqrt(2*pi_)*sqrt(fabs(S.determinant())))*exp(-0.5* Zminus.transpose() * S.inverse() * Zminus);
		model_ita[i] = ita;
	}

	float c_temp = 0;
	for(int i=0; i<model_size; ++i){
		c_temp += model_ita[i] * c_(i);
	}

	for(int i=0; i<model_size; ++i){
		model_pro_(i) = (model_ita[i]*c_(i))/c_temp;
		if(model_pro_(i)<1e-4) model_pro_(i) = 1e-4;
	}

	filewrite<<Z(0)<<" "<<Z(1)<<" ";
	MixProbability();

	//td::cout<<"########## Model probability ##########\n "<<model_pro_<<std::endl;
	//std::cout<<"############################## IMM UpdateProbability END #############################"<<std::endl;
}

//输出交互
void IMM_UKF::MixProbability(){
	//std::cout<<"########## IMM MixProbability #########"<<std::endl;
	x_merge_.fill(0.0);
	p_merge_.fill(0.0);
	for(int i=0; i<model_size; ++i){
		model_X_[i] = imm_ukf_[i].Get_state();//当前时刻更新结果
		model_P_[i] = imm_ukf_[i].Get_covariance();
		//std::cout<<" model_X_ \n"<<model_X_[i]<<"\n"<<std::endl;
		x_merge_ += model_X_[i] * model_pro_(i);
	}

	//std::cout<<"########## final merge X ##########\n"<< x_merge_<<std::endl;
	for(int i=0; i<model_size; ++i){
		p_merge_ += model_pro_(i) * (model_P_[i] + (model_X_[i] -x_merge_)* (model_X_[i] -x_merge_).transpose());
	}
	filewrite<<track_id_<<" "<<x_merge_(0)<<" "<<x_merge_(1)<<" "<<x_merge_(2)<<" "<<x_merge_(3)<<" "<<model_pro_(0)<<" "<<model_pro_(1)<<" "<<model_pro_(2)<<"\n";
	//std::cout<<"final merge P\n"<< p_merge_<<std::endl;
}


void IMM_UKF::Process(std::vector<Eigen::VectorXd>& Z, const Eigen::VectorXd& beta, const float& last_beta, float& time){
	//std::cout<<"#################### IMM Process ####################"<<std::endl;
	if(!IsInitialized()){
		IMM_Initialization(Z[0], time,0,0);
	}else{
		PredictionZmerge(time);
		UpdateProbability(Z, beta, last_beta);
	}
}


Eigen::VectorXd IMM_UKF::getMixState(){
	return x_merge_;
}

Eigen::MatrixXd IMM_UKF::getMixCovariance(){

	return p_merge_;
}

Eigen::MatrixXd IMM_UKF::GetS(){
	return S_merge_;
}

Eigen::VectorXd IMM_UKF::GetZpre(){
	return Zpre_merge_;
}
