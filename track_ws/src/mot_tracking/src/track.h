/*
 * author: wx
 * date: 2020.12.13
 * reference:
 */
#ifndef TRACK_H
#define TRACK_H

#include<string>
#include<iostream>
#include<stdlib.h>
#include<vector>

//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include<Eigen/Dense>

#include"imm_ukf.h"
#include"readparam.h"

struct Detect{
	int classname = 0;
	float score;//detection score
	float z;
	float yaw;
	Eigen::VectorXd position;//x,y
	std::vector<float> box;//3D box in lidar h、w、l
	std::vector<float> box2D;//2D box in camera left top point and right down point
	cv::RotatedRect rotbox;
};

struct Box{
	float yaw;
	float length;
	float width;
	float height;
	float z;
};

struct TrackState{
	int Confirmed = 1;
	int UnConfirmed = 2;
	int Delete = 3;
};


class Track{
public:
	//TODO beta
	Track(Param& p, int id, float time, Detect& det, float velocity, float angle):param_(p),id_(id){

		imm_ukf_=std::shared_ptr<IMM_UKF>(new IMM_UKF(id, param_.pmodel, param_.pstate_v, param_.pmea_v, param_.pP, param_.pQ, param_.pR, param_.pinteract));

		Eigen::VectorXd z(2);
		//z.fill(0.0);
		float x = det.position(0);
		float y = det.position(1);
		boundingbox.yaw = det.yaw;
		boundingbox.width = det.box[1];
		boundingbox.length = det.box[2];
		boundingbox.height = det.box[0];
		boundingbox.z = det.z;
		z<<x,y;
		imm_ukf_->IMM_Initialization(z,time,velocity, angle);
		track_state_ = Track_state_.UnConfirmed;
		age_ = 0;
		measure_<<x,y;
	};

	~Track(){};

	int GetId(){
		return id_;
	}

	int Prediction(float time){
		age_++;
		/*if(age_> 1){
			track_state_ = Track_state_.UnConfirmed;
		}*/
		imm_ukf_->PredictionZmerge(time);	
	}

	int Update(std::vector<Eigen::VectorXd>& det, const Eigen::VectorXd& beta, const float& last_beta, float& time){
		imm_ukf_->UpdateProbability(det, beta, last_beta);
		age_ = 0;
		hit_ += 1;
		if(track_state_ == Track_state_.UnConfirmed || hit_ > max_init_){
			track_state_ = Track_state_.Confirmed;
		}
	}

	int Update(Eigen::VectorXd& det){
		imm_ukf_->UpdateProbability(det);
		age_ = 0;
		hit_ += 1;
		if(track_state_ == Track_state_.UnConfirmed || hit_ > max_init_){
			track_state_ = Track_state_.Confirmed;
		}
	}

	int Age(){
		return age_;
	}

	//漏检后的跟踪状态转移
	void MarkMissed(){
		//std::cout<<"MarkMissed "<<track_state_<<std::endl;

		if(track_state_ == Track_state_.UnConfirmed){
			track_state_ = Track_state_.Delete;
			//std::cout<<"track_state_ "<<track_state_<<std::endl;
		}else if(age_ > max_age_){
			//std::cout<<"age "<<age_<<std::endl;
			track_state_ = Track_state_.Delete;
		}
	}

	//获取track跟踪的状态
	int GetTrackState(){
		return track_state_;
	}

	//获取目标的状态向量
	Eigen::VectorXd GetState(){
		 Eigen::VectorXd state = imm_ukf_->getMixState();
		 for(int i=0; i<6; ++i){
			 if(std::isnan(state(i))){
				 track_state_ = Track_state_.Delete;
			 }
		 }
		 return state;
	}

	Eigen::MatrixXd S(){
		Eigen::MatrixXd S = imm_ukf_->GetS();
		return S;
	}

	Eigen::VectorXd GetZ(){
		Eigen::VectorXd Z = imm_ukf_->GetZpre();
		return Z;
	}

	void UpdateBox(Detect& det){
		if(fabs(det.yaw - boundingbox.yaw)<(20*M_PI/180)){
			boundingbox.yaw = det.yaw;
			boundingbox.width = det.box[1];
			boundingbox.length = det.box[2];
			boundingbox.height = det.box[0];
			boundingbox.z = det.z;
		}
	}

	Box GetBox(){
		return boundingbox;
	}

	Eigen::Vector2f GetMeasure(){
		return measure_;
	}

	void UpdateMeasure(float x, float y){
		measure_(0) = x;
		measure_(1) = y;
	}


private:
	std::shared_ptr<IMM_UKF> imm_ukf_;
	Param param_;
	int id_ = -1;
	int age_ = 0;//表示距离上次update的时间
	int max_age_ = 4;
	int hit_ = 1;//跟踪次数
	int max_init_ = 3;
	int track_state_ = 0;
	Eigen::Vector2f measure_;
	TrackState Track_state_;
	Box boundingbox;//检测对应目标
};

#endif
