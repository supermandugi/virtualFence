#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <fstream>
#include <cmath>
#include <cfloat>

#include "ros/ros.h"
#include "std_msgs/String.h"



class MouseInterface
{
private:
	int number_of_mouse;

public:
	MouseInterface(int maxMouse_num);
	
	~MouseInterface();

	struct mouse_info
	{
		int x = 0, y = 0, click = 0;
		int change_event;


	};
	mouse_info* Mouse;
	void setWindow(char* name, int mouse_idx);
};
MouseInterface::MouseInterface(int maxMouse_num = 1)
{
	number_of_mouse = maxMouse_num;
	Mouse = new mouse_info[maxMouse_num];
}
MouseInterface::~MouseInterface()
{
	delete[] Mouse;
}
void onMouse(int event, int x, int y, int, void* param)
{
	MouseInterface::mouse_info *input_mouse = (MouseInterface::mouse_info *)param;

	input_mouse->x = x;
	input_mouse->y = y;
	input_mouse->click = event;

	if (event != 0 && (input_mouse->change_event != input_mouse->click))
		input_mouse->change_event = event;
};
void MouseInterface::setWindow(char* name, int mouse_idx)
{
	if (mouse_idx<0 || mouse_idx>this->number_of_mouse - 1)
	{
		printf("There is no %d mouse\n", mouse_idx);
		return;
	}

	cv::setMouseCallback(name, onMouse, &this->Mouse[mouse_idx]);
}

struct NodeInfo
{
	int idx = -1;
	cv::Point pose;

	std::vector<std::pair<int, double>> imgs; // idx, angle
};

class MapBuilder
{
private:
	std::string windowName;

	struct Pram
	{
		int img_margin = 40;
		int img_node_gap = 80;

		double rorationSpeed = 90.0;
		double real_360_angle = 360.0;
		double img_number = 12.0;
		double arrow_size = 20.0;
		double minRotVel = 70.0;
		double maxVel = 200.0;
		double maxDistance = 100.0;
		cv::Point center;

		double nearGoalDist = 40.0;
	};
	struct Output
	{
		double angle;
		double ave_likelihood;

		double targetAngle;
		double targetDistance;

		cv::Point2f meanPose;
		cv::Point2f poseVariance;

		double goalDist;
	};
	struct Input
	{
		cv::Point2f Obstacle_vector;
	};

	int isMaking = 0;
	int isRunning = 0;

	int r, c;

	int TotalNodeNum = 0;
	double angle_gap = 0;
	double base[3];
	cv::Mat plot_ori, plot;

	std::map<int, NodeInfo> nodes; // node Idx
	std::map<int, NodeInfo>::iterator target;

	std::map<int, int> img_DB; // img idx, node Idx

	void DrawNode(cv::Mat &outputimg);
	void pickNode(void);
	void nodeMake(void);
	void generateForce(cv::Mat &output_img);
	void generateMotion(void);
	void checkGoalDist(void);
	void drving(void);

	void save(void);
	void load(void);

	void tempSave(std::string path);
	void tempLoad(std::string path);

	MouseInterface *mouse = nullptr;
	cv::VideoWriter oVideoWriter;

public:
	Robot *robotptr;
	ObservationData *obptr;
	Kinect2Data *kinect2ptr;

	MapBuilder(int row, int col);
	~MapBuilder(void);

	bool isObstacleDetected = 0;
	Pram pram;
	Input input;
	Output output;

	double getAngle(int img_idx);
	void run(void);
};
MapBuilder::MapBuilder(int row, int col)
{
	windowName = "Map";

	mouse = new MouseInterface(1);

	int margin = pram.img_margin;
	int node_gap = pram.img_node_gap;
	plot_ori = cv::Mat(2 * margin + (row - 1) * node_gap, 2 * margin + (row - 1) * node_gap, CV_8UC3, cv::Scalar(255, 255, 255));

	pram.center = cv::Point(plot_ori.cols / 2, plot_ori.rows / 2);

	r = row;
	c = col;
	TotalNodeNum = row * col;

	for (int i = 0; i < TotalNodeNum; i++)
	{
		NodeInfo temp;
		temp.idx = i;

		int r = i / col;
		int c = i % col;
		temp.pose = cv::Point(margin + node_gap * c, margin + node_gap * r);
		cv::circle(plot_ori, temp.pose, margin / 2, cv::Scalar(0, 255, 0), 1);

		nodes.insert(std::pair<int, NodeInfo>(i, temp));
	}

	target = nodes.end();

	//oVideoWriter =  cv::VideoWriter("map.avi", CV_FOURCC('P', 'I', 'M', '1'), 20, cv::Size(obptr->col, obptr->row) , true);

	srand((unsigned)time(NULL));
}
MapBuilder::~MapBuilder(void)
{
	if (mouse) delete mouse;
}

void MapBuilder::DrawNode(cv::Mat &outputimg)
{
	if ((int)img_DB.size() != obptr->num_DB_size) return;

	std::map<int, NodeInfo>::iterator n;

	float li_sum = 0.0f;
	output.meanPose = cv::Point2f(0.0f, 0.0f);

	for (n = nodes.begin(); n != nodes.end(); n++)
	{
		NodeInfo &_node = n->second;
		for (int i = 0; i < (int)_node.imgs.size(); i++)
		{
			if (_node.imgs.at(i).first == -1) continue;

			double val = (double)obptr->likelihood[_node.imgs.at(i).first];

			double angle_radian = CV_PI / 180.0 * _node.imgs.at(i).second;
			double x = val * pram.arrow_size * cos(angle_radian);
			double y = val * pram.arrow_size * sin(angle_radian);

			if (val > 1.0f)
			{
				li_sum += (float)val;
				output.meanPose.x += (float)val * (float)_node.pose.x;
				output.meanPose.y += (float)val * (float)_node.pose.y;
			}

			cv::Point tip(_node.pose.x - (int)y, _node.pose.y - (int)x);
			cv::line(outputimg, _node.pose, tip, cv::Scalar(0, 0, 0));
			cv::putText(outputimg, std::to_string(_node.imgs.at(i).first), tip, 0, 0.3, cv::Scalar(0, 0, 0));
		}
	}
	
	//output.meanPose = output.meanPose / tmp;
	output.meanPose.x = output.meanPose.x / li_sum;
	output.meanPose.y = output.meanPose.y /li_sum;
	output.poseVariance = cv::Point2f(0.0f, 0.0f);
	for (n = nodes.begin(); n != nodes.end(); n++)
	{
		NodeInfo &_node = n->second;
		for (int i = 0; i < (int)_node.imgs.size(); i++)
		{
			if (_node.imgs.at(i).first == -1) continue;

			double val = (double)obptr->likelihood[_node.imgs.at(i).first];

			if (val > 1.0f)
			{
				output.poseVariance.x += val * ((float)_node.pose.x - output.meanPose.x) * ((float)_node.pose.x - output.meanPose.x);
				output.poseVariance.y += val * ((float)_node.pose.y - output.meanPose.y) * ((float)_node.pose.y - output.meanPose.y);
			}
		}
	}
	//output.poseVariance = output.poseVariance / li_sum;
	output.poseVariance.x / li_sum;
	output.poseVariance.y /li_sum;

	if (li_sum != 0 && std::isfinite(output.meanPose.x))
	{
		cv::circle(outputimg, output.meanPose, 30, cv::Scalar(255, 0, 0), 2);
		cv::ellipse(outputimg, cv::Point(output.meanPose.x, output.meanPose.y)
			, cv::Size(std::sqrt(output.poseVariance.x), std::sqrt(output.poseVariance.y)), 0.0
			, 0.0, 360.0, cv::Scalar(255, 0, 0));
	}
}
void MapBuilder::pickNode(void)
{
	if (isMaking != 1) return;

	angle_gap = 360.0 / (double)pram.img_number;

	mouse->setWindow((char *)windowName.c_str(), 0);

	std::map<int, NodeInfo>::iterator i;

	target = nodes.end();

	double distance = -1.0;
	for (i = nodes.begin(); i != nodes.end(); i++)
	{
		int x_gap = mouse->Mouse[0].x - i->second.pose.x;
		int y_gap = mouse->Mouse[0].y - i->second.pose.y;
		double dist = std::sqrt(x_gap*x_gap + y_gap*y_gap);

		if (dist < distance || distance < 0)
		{
			distance = dist;
			target = i;
		}
	}

	robotptr->set_vel = 0.0;
	robotptr->set_rotvel = 0.0;
}
void MapBuilder::nodeMake(void)
{
	if (isMaking != 2) return;
	if (target == nodes.end()) return;

	robotptr->set_vel = 0;
	robotptr->set_rotvel = pram.rorationSpeed * 0.7;

	double error_motion_th = robotptr->th - base[2];
	double motion_th = (360.0 / pram.real_360_angle) * error_motion_th;

	if (motion_th < 0) motion_th += 360.0;

	int angle_idx = (int)(motion_th / angle_gap);
	double rate = motion_th / angle_gap - (double)angle_idx;
	double temp_angle_gap = rate * angle_gap;
	
	if (abs(temp_angle_gap) < 5 && target->second.imgs.at(angle_idx).first == -1)
	{
		robotptr->set_rotvel = 0;
		//Sleep(1000);
		int img_idx = (int)img_DB.size();

		target->second.imgs.at(angle_idx).first = img_idx;
		target->second.imgs.at(angle_idx).second = motion_th;

		//target->second.pose = cv::Point(robotptr->x, robotptr->y);

		img_DB.insert(std::pair<int, int>(img_idx, target->first));
		
		cv::Mat RGB = cv::Mat(obptr->row, obptr->col, CV_8UC3, cv::Scalar(0));
		std::memcpy(RGB.data, obptr->RGB_data, sizeof(char) * 3 * obptr->row * obptr->col);

		cv::imwrite(cv::format("image%d_%d.jpg", target->first, angle_idx), RGB);
		cv::imwrite(cv::format("image_plot%d_%d.jpg", target->first, angle_idx), obptr->line_plot.clone());

		obptr->mode = 1;
	}

	bool isFinish = true;
	for (int i = 0; i < pram.img_number; i++)
	{
		if (target->second.imgs.at(i).first == -1) isFinish = false;
	}

	if (isFinish)
	{
		isMaking = 0;
		robotptr->set_vel = 0.0;
		robotptr->set_rotvel = 0.0;
		target = nodes.end();
	}

}
double MapBuilder::getAngle(int img_idx)
{
	if (img_idx < 0 || img_idx >= (int)img_DB.size()) return NULL;

	std::map<int, int>::iterator node_finder = img_DB.find(img_idx);
	if (node_finder == img_DB.end()) return NULL;

	int node_idx = node_finder->second;

	std::map<int, NodeInfo>::iterator node_ptr = nodes.find(node_idx);
	if (node_ptr == nodes.end()) return NULL;

	int i = 0;
	double img_angle = 0.0;
	for (; i < (int)node_ptr->second.imgs.size(); i++)
		if (img_idx == node_ptr->second.imgs.at(i).first)
		{
			img_angle = node_ptr->second.imgs.at(i).second;
			break;
		}

	if (i == (int)node_ptr->second.imgs.size()) return NULL;

	cv::Point gap = node_ptr->second.pose - pram.center;

	double node_angle = 180.0 / CV_PI * atan2((double)gap.x, (double)gap.y);
	double result = node_angle - img_angle;
	if (result > 180.0) result -= 360.0;
	if (result <= -180.0) result += 360.0;

	return result;
}
void MapBuilder::generateForce(cv::Mat &output_img)
{
	if (obptr->num_DB_size != (int)img_DB.size()) return;

	float *likelihood = new float[img_DB.size()];
	//obptr->mtx.lock();
	std::memcpy(likelihood, obptr->likelihood, sizeof(float) * img_DB.size());
	//obptr->mtx.unlock();

	int counter = 0;
	double sum_x = 0, sum_y = 0, sum_li = 0;
	for (int i = 0; i < obptr->num_DB_size; i++)
	{
		if (likelihood[i] <= 1.0f) continue;

		double angle = getAngle(i);
		if (!std::isfinite(angle)) continue;

		double radian = CV_PI / 180.0 * angle;
		double x = (double)likelihood[i] * cos(radian);
		double y = (double)likelihood[i] * sin(radian);

		sum_x += x;
		sum_y += y;
		sum_li += (double)likelihood[i];
		counter++;
	}

	output.angle = 0.0;
	output.ave_likelihood = 0.0;

	if (counter == 0) return;

	output.ave_likelihood = sum_li / (double)counter;

	double output_radian = atan2(sum_y, sum_x);

	//printf("avg_xy : %lf, %lf\n", sum_x / (double)counter, sum_y / (double)counter);

	output.angle = 180.0 / CV_PI * output_radian;

	cv::Point tip = pram.center - cv::Point(output.ave_likelihood * pram.arrow_size * sin(output_radian), output.ave_likelihood * pram.arrow_size * cos(output_radian));
	cv::line(output_img, cv::Point(pram.center.x, pram.center.y), tip, cv::Scalar(0, 0, 255), 3);

	delete[] likelihood;
}
void MapBuilder::generateMotion(void)
{
	if (isRunning == 0 && isMaking == 0)
	{
		robotptr->set_vel = 0.0;
		robotptr->set_rotvel = 0.0;
	}

	if (isRunning == 1)
	{// ¸ñÇ¥ ¼³Á¤
		output.targetAngle = output.angle + robotptr->th;

		if (output.targetAngle > 180.0) output.targetAngle -= 360.0;
		if (output.targetAngle <= -180.0) output.targetAngle += 360.0;

		output.targetDistance = std::min(output.ave_likelihood / 3.0, 1.0) * pram.maxDistance;

		if (output.ave_likelihood == 0.0)
		{
			output.targetAngle = (double)rand() / RAND_MAX * 360.0 - 180.0;
			output.targetDistance = pram.maxDistance / 2.0;

			printf("rand angle : %lf\n", output.targetAngle);
		}

		printf("output.angle : %lf\n\n", output.angle);

		isRunning = 2;
	}

	if (isRunning == 2)
	{// Á¦ÀÚ¸® È¸Àü
		double gap_angle = output.targetAngle - robotptr->th;
		if (gap_angle > 180.0) gap_angle -= 360.0;
		if (gap_angle <= -180.0) gap_angle += 360.0;

		double rotvel = gap_angle;
		if (abs(rotvel) < pram.minRotVel)
		{
			if (rotvel < 0) rotvel = -pram.minRotVel;
			else rotvel = pram.minRotVel;
		}

		//printf("output.rotvel : %lf\n\n", rotvel);

		robotptr->set_vel = 0.0;
		robotptr->set_rotvel = rotvel;

		if (abs(gap_angle) < 5)
		{
			base[0] = robotptr->x;
			base[1] = robotptr->y;
			base[2] = robotptr->th;
			isRunning = 3;
		}

	}

	if (isRunning == 3)
	{//Á÷Áø
		robotptr->set_vel = pram.maxVel;
		robotptr->set_rotvel = 0.0;

		double x_gap = robotptr->x - base[0];
		double y_gap = robotptr->y - base[1];

		double moving_dist = std::sqrt(x_gap*x_gap + y_gap*y_gap);
		if (moving_dist > output.targetDistance)
			isRunning = 1;
	}


	if (isRunning == 5) 
	{
		int x = robotptr->x;
		int y = robotptr->y;
		double distance = sqrt(x*x + y*y);
		//std::cout << sqrt(x*x + y*y) << std::endl;

		//robotptr->set_vel = std::max(distance, 50.0);
		double target = 180.0 / CV_PI * atan2(y, x) + 180;
		//std::cout << x << "," << y << "->" << target << std::endl;
		robotptr->set_vel = 0;
		robotptr->set_rotvel = 0;
		robotptr->set_rotvel = target - robotptr->th;
		std::cout << robotptr->get_rotvel << std::endl;
	}

}
void MapBuilder::checkGoalDist(void)
{
	float x_gap = (float)pram.center.x - output.meanPose.x;
	float y_gap = (float)pram.center.y - output.meanPose.y;
	output.goalDist = std::sqrt(x_gap*x_gap + y_gap*y_gap);

	if (output.goalDist < (float)pram.nearGoalDist)
	{
		if (isRunning == 9)
			isRunning = 8;
	}
}
void MapBuilder::drving(void)
{
	if (isMaking != 0) return;
	char ch = 0;
	ch = getchar();
	if (ch =='d') if (isRunning == 0) isRunning = 8;

	if (isRunning == 8)
	{
		std::vector<std::map<int, NodeInfo>::iterator> vaildNodes;
		std::map<int, NodeInfo>::iterator i = nodes.begin();
		for (; i != nodes.end(); i++)
			if (i->second.imgs.size() != 0) vaildNodes.push_back(i);

		if (vaildNodes.size() != 0)
		{
			int rand_idx = rand() % vaildNodes.size();
			rand_idx = (vaildNodes.size()-1)/2;

			pram.center = vaildNodes.at(rand_idx)->second.pose;

			if (isObstacleDetected)
			{
				pram.center.x = (int)(((float)pram.center.x + input.Obstacle_vector.x) / 2.0f);
			}
		}

		isRunning = 9;
	}

	if (isRunning == 9)
	{
		double weight = 100.0;
		double rot_vel = (weight * robotptr->set_rotvel + output.angle) / (weight + 1.0);
		robotptr->set_rotvel = rot_vel;

		double variance = 3600;
		double vel_rate = exp(-(rot_vel*rot_vel) / variance);


		robotptr->set_vel = 300.0 * vel_rate;
	}
}
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data);
	std::cout << "Heard : " << msg->data.c_str() << std::endl;
}
void MapBuilder::run(void)
{
	char **cz
	ros::init(0,cz,"listener");
	ros::NodeHandle nz;
	ros::Subscriber subz = nz.subscribe("comm", 1000, chatterCallback);
	ros::spin();
	mouse->setWindow((char *)windowName.c_str(), 0);

	if (mouse->Mouse[0].change_event == 1)
	{
		pram.center.x = mouse->Mouse[0].x;
		pram.center.y = mouse->Mouse[0].y;
	}

	plot = plot_ori.clone();
	cv::line(plot, cv::Point(0, pram.center.y), cv::Point(plot_ori.cols, pram.center.y), cv::Scalar(0, 0, 255));
	cv::line(plot, cv::Point(pram.center.x, 0), cv::Point(pram.center.x, plot_ori.rows), cv::Scalar(0, 0, 255));

	// b key ³ëµå ¼±ÅÃ
	if (getchar() == 'b')
	{
		if (isMaking == 0)
		{
			target = nodes.end();
			isMaking = 1;
			isRunning = 0;
		}

	}

	// space key ¼±ÅÃµÈ ³ëµå¿¡ data »ðÀÔ
	if (getchar() == 'z')
	{
		if (isMaking == 1 && target != nodes.end())
		{
			base[0] = robotptr->x;
			base[1] = robotptr->y;
			base[2] = robotptr->th;

			if (target->second.imgs.size() != 0)target->second.imgs.clear();
			for (int i = 0; i < pram.img_number; i++)
				target->second.imgs.push_back(std::pair<int, double>(-1, angle_gap * (double)i));

			isMaking = 2;
			isRunning = 0;
		}
	}

	// n key ¸ð¼Ç »ý¼º
	if (getchar() == 'n')
	{
		if (isMaking == 0 && isRunning == 0)
			isRunning = 1;

		if (isRunning == 9) isRunning = 1;
	}

	// s key ÀúÀå
	if (getchar() == 's')
	{
		if (isMaking == 0 && isRunning == 0)
		{
			//save();
			tempSave("..\\data\\MapData.dat");
			obptr->mode = 2;
		}
	}

	// l key ·Îµå
	if (getchar() == 'l')
	{
		isMaking = 0;
		isRunning = 0;

		//load();
		tempLoad("..\\data\\MapData.dat");
		obptr->mode = 3;
	}

	// t key  test
	if (getchar() == 't')
	{
		if (isMaking == 0 && isRunning == 0)
			isRunning = 5;

		if (isRunning == 9) isRunning = 5;
	}

	if (isMaking == 1)
	{
		pickNode();
	}
	if (isMaking == 2)
	{
		nodeMake();
	}

	if (isObstacleDetected)
	{
		if (isRunning == 3)
			isRunning = 1;

		if (isRunning == 9)
			isRunning = 8;
	}

	generateForce(plot);
	generateMotion();
	drving();

	if (target != nodes.end()) cv::circle(plot, target->second.pose, 20, cv::Scalar(0, 255, 0), 10);
	DrawNode(plot);
	cv::imshow(windowName, plot);
	//oVideoWriter();
	

	checkGoalDist();
}
void MapBuilder::save(void)
{
	int nodes_size = (int)nodes.size();
	int NumberofImgsize;
	/////////////////<save nodes>///////////////////
	cv::FileStorage file("..\\data\\nodes.yml", cv::FileStorage::WRITE);

	std::cout << "save flag : " << file.isOpened() << std::endl;

	if (!file.isOpened()) return;

	cv::write(file, "Nodesize", nodes_size);
	std::map<int, NodeInfo>::iterator i = nodes.begin();
	int count = 0;
	for (; i != nodes.end(); i++) //25
	{
		cv::write(file, cv::format("Nodeidx_%d", count), i->first);
		cv::write(file, cv::format("ImageIdxPerNode_%d", count), i->second.idx);
		cv::write(file, cv::format("pose_x_%d", count), i->second.pose.x);
		cv::write(file, cv::format("pose_y_%d", count), i->second.pose.y);
		NumberofImgsize = (int)i->second.imgs.size();
		cv::write(file, cv::format("Number_of_Img_%d", count), NumberofImgsize);
		for (int img_i = 0; img_i < (int)i->second.imgs.size(); img_i++) //12
		{
			cv::write(file, cv::format("Imgs_idx_%d", img_i), i->second.imgs.at(img_i).first);
			cv::write(file, cv::format("Imgs_angle_%d", img_i), i->second.imgs.at(img_i).second);
		}
		count++;
	}

	/////////////////<save img_DB>///////////////////
	int img_DB_size = (int)img_DB.size();
	cv::FileStorage file_1("../data/img_DB.yml", cv::FileStorage::WRITE);
	cv::write(file_1, "img_DB_size", img_DB_size);
	std::map<int, int>::iterator j = img_DB.begin();
	count = 0;
	for (; j != img_DB.end(); j++)
	{
		cv::write(file_1, cv::format("img_DB_first_%d", count), j->first);
		cv::write(file_1, cv::format("img_DB_second_%d", count), j->second);
		count++;
	}

	file.release();
	file_1.release();
}
void MapBuilder::load(void)
{
	nodes.clear();
	img_DB.clear();

	int nodes_size_load;
	int nodes_first;
	int imgs_size;
	int imgs_first;
	double imgs_second;
	int img_DB_size_load;
	int img_DB_first;
	int img_DB_second;

	NodeInfo temp;

	/////////////////<load nodes>///////////////////
	//cv::FileStorage file("../data/nodes.yml", cv::FileStorage::READ);

	cv::FileStorage file;
	file.open("..\\data\\nodes.yml", cv::FileStorage::READ);

	std::cout << file.isOpened() << std::endl;

	if (!file.isOpened()) return;


	nodes_size_load = (int)file["Nodesize"];

	for (int i = 0; i < nodes_size_load; i++)
	{
		nodes_first = file[cv::format("Nodeidx_%d", i)];
		temp.idx = file[cv::format("ImageIdxPerNode_%d", i)];
		temp.pose.x = file[cv::format("pose_x_%d", i)];
		temp.pose.y = file[cv::format("pose_y_%d", i)];
		imgs_size = (int)file[cv::format("Number_of_Img_%d", i)];

		for (int img_i = 0; img_i < imgs_size; img_i++)
		{
			imgs_first = file[cv::format("Imgs_idx_%d", img_i)];
			imgs_second = file[cv::format("Imgs_angle_%d", img_i)];
			temp.imgs.push_back(std::pair<int, double>(imgs_first, imgs_second));
		}
		nodes.insert(std::pair<int, NodeInfo>(nodes_first, temp));
	}

	/////////////////<load img_DB>///////////////////
	cv::FileStorage file_1;
	file_1.open("../data/img_DB.yml", cv::FileStorage::READ);
	img_DB_size_load = (int)file_1["img_DB_size"];

	for (int i = 0; i < img_DB_size_load; i++)
	{
		img_DB_first = file[cv::format("img_DB_first_%d", i)];
		img_DB_second = file[cv::format("img_DB_second_%d", i)];
		img_DB.insert(std::pair<int, int>(img_DB_first, img_DB_second));
	}

}
void MapBuilder::tempSave(std::string path)
{
	std::ofstream MapDBsaver(path, std::ios_base::out | std::ios_base::trunc | std::ios_base::binary);
	if (!MapDBsaver.is_open())
	{
		std::cout << "Can not open " << path << std::endl;
		return;
	}

	MapDBsaver.write((char*)&r, sizeof(int));
	MapDBsaver.write((char*)&c, sizeof(int));

	int Nodes_size = nodes.size();
	MapDBsaver.write((char*)&Nodes_size, sizeof(int));

	std::map<int, NodeInfo>::iterator n;
	for (n = nodes.begin(); n != nodes.end(); n++)
	{
		MapDBsaver.write((char*)&n->first, sizeof(int));
		MapDBsaver.write((char*)&n->second.idx, sizeof(int));
		MapDBsaver.write((char*)&n->second.pose.x, sizeof(int));
		MapDBsaver.write((char*)&n->second.pose.y, sizeof(int));

		int img_size = (int)n->second.imgs.size();
		MapDBsaver.write((char*)&img_size, sizeof(int));

		std::vector<std::pair<int, double>>::iterator i;
		for (i = n->second.imgs.begin(); i != n->second.imgs.end(); i++)
		{
			MapDBsaver.write((char*)&i->first, sizeof(int));
			MapDBsaver.write((char*)&i->second, sizeof(double));
		}
	}

	int img_DB_size = (int)img_DB.size();
	MapDBsaver.write((char*)&img_DB_size, sizeof(int));

	std::map<int, int>::iterator img_i;
	for (img_i = img_DB.begin(); img_i != img_DB.end(); img_i++)
	{
		MapDBsaver.write((char*)&img_i->first, sizeof(int));
		MapDBsaver.write((char*)&img_i->second, sizeof(int));
	}

	std::cout << "Save is complete" << std::endl;
	MapDBsaver.close();
}
void MapBuilder::tempLoad(std::string path)
{
	std::ifstream MapDBloader(path, std::ios_base::in | std::ios_base::binary);
	if (!MapDBloader.is_open())
	{
		std::cout << "Can not open " << path << std::endl;
		return;
	}

	nodes.clear();
	img_DB.clear();

	MapDBloader.read((char*)&r, sizeof(int));
	MapDBloader.read((char*)&c, sizeof(int));

	int margin = pram.img_margin;
	int node_gap = pram.img_node_gap;
	plot_ori = cv::Mat(2 * margin + (r - 1) * node_gap, 2 * margin + (r - 1) * node_gap, CV_8UC3, cv::Scalar(255, 255, 255));

	int DB_size;
	MapDBloader.read((char*)&DB_size, sizeof(int));
	
	for (int i = 0; i < DB_size; i++)
	{
		int first;
		NodeInfo temp;

		MapDBloader.read((char*)&first, sizeof(int));
		MapDBloader.read((char*)&temp.idx, sizeof(int));
		MapDBloader.read((char*)&temp.pose.x, sizeof(int));
		MapDBloader.read((char*)&temp.pose.y, sizeof(int));

		int img_size;
		MapDBloader.read((char*)&img_size, sizeof(int));
		for (int j = 0; j < img_size; j++)
		{
			int img_first;
			double img_second;
			MapDBloader.read((char*)&img_first, sizeof(int));
			MapDBloader.read((char*)&img_second, sizeof(double));

			temp.imgs.push_back(std::pair<int, double>(img_first, img_second));
		}

		cv::circle(plot_ori, temp.pose, margin / 2, cv::Scalar(0, 255, 0), 1);

		nodes.insert(std::pair<int, NodeInfo>(first, temp));
	}

	int img_DB_size;
	MapDBloader.read((char*)&img_DB_size, sizeof(int));
	for (int i = 0; i < img_DB_size; i++)
	{
		int first, second;
		MapDBloader.read((char*)&first, sizeof(int));
		MapDBloader.read((char*)&second, sizeof(int));

		img_DB.insert(std::pair<int, int>(first, second));
	}

	std::cout << "Load is complete" << std::endl;
	MapDBloader.close();
}