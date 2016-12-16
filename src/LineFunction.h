#pragma once
#include <fstream>
#include "NavigationFunction.h"

class Tree
{
private:
	struct Pram
	{
		int BRANCH_FACTOR; //K
		int LEVEL; //L
		int DESC_DIM; //D

		float stddev_threshold = 2.0f;
		float likelihood_rate = 1.0f;
		bool use_virtual_doc = 1;
	};
	struct int_node_t
	{
		float *c;

		//int_node_t::int_node_t(int K, int D)
		//{
		//	c = new float[K*D];
		//};
		//int_node_t::~int_node_t()
		//{
		//	delete[] c;
		//};
	};
	struct leaf_node_t { std::map<int, float> doc; };
	Pram pram;

	size_t _num_int, _num_leaf;
	int_node_t *_int_node;
	leaf_node_t *_leaf_node;

	void free(void);
	size_t child_idx(size_t i, int j = 0) { return i*pram.BRANCH_FACTOR + j + 1; };
	size_t leaf_idx(size_t i) const { return i - _num_int; };
	float dist_func(const float *f0, const float *f1);
	size_t find_leaf(cv::Mat &_disc);

public:
	int doc_size = 0;
	int total_desc = 0;

	Tree(int branch_factor, int level, int desc_dim)
	{
		pram.BRANCH_FACTOR = branch_factor;
		pram.LEVEL = level;
		pram.DESC_DIM = desc_dim;
	};
	int load_tree_structure(std::string _path);
	int save_tree_DB(std::string _path);
	int load_tree_DB(std::string _path);
	bool insert_doc(int doc_id, cv::Mat &disc);
	void query_doc(cv::Mat &_disc, float *score_, float *likeli_);
	cv::Mat drawDistribution(cv::Mat *input, float nomalize = 0.0f);
};
void Tree::free(void)
{
	for (size_t i = 0; i < _num_int; ++i)
		delete[] _int_node[i].c;

	delete[] _int_node;
	delete[] _leaf_node;
}
int Tree::load_tree_structure(std::string _path)
{
	FILE *fp = fopen(_path.c_str(), "rb");
	//fopen_s(&fp, _path.c_str(), "rb"); // edit by yn
	if (fp == NULL) {
		printf("No such file: %s\n", _path.c_str());
		return 0;
	}
	//free();

	int D_K = pram.BRANCH_FACTOR*pram.DESC_DIM;

	size_t num_node = 1, num_node_level = 1;
	for (int l = 1; l<pram.LEVEL; ++l)
		num_node += (num_node_level *= pram.BRANCH_FACTOR);

	_num_int = num_node;  //_num_node<L-1>(K);
	_num_leaf = (num_node += (num_node_level *= pram.BRANCH_FACTOR)) - _num_int; //_num_node<L>(K) - _num_int;
																				 //printf("voctree: num_int: %d (%d), num_leaf: %d\n"
																				 //	, (int)_num_int, (int)(_num_int*D_K*sizeof(float)), (int)_num_leaf);

	_int_node = new int_node_t[_num_int];
	_leaf_node = new leaf_node_t[_num_leaf];

	for (size_t i = 0; i<_num_int; ++i)
	{
		_int_node[i].c = new float[D_K];
		if (fread(_int_node[i].c, sizeof(float), D_K, fp) < D_K)
		{
			fclose(fp);
			return 0;
		}
	}

	fclose(fp);
	doc_size = 0;

	return 1;
}
int Tree::save_tree_DB(std::string _path)
{
	std::ofstream treeDBsaver(_path.c_str(), std::ios_base::out | std::ios_base::trunc | std::ios_base::binary);
	if (!treeDBsaver.is_open())
	{
		printf("Cannot open TreeDB file!\n");
		return 0;
	}

	treeDBsaver.write((char*)&doc_size, sizeof(int));
	treeDBsaver.write((char*)&total_desc, sizeof(int));
	int leaf_cnt = 0;


	for (int i = 0; i<(int)_num_leaf; ++i)
	{
		const leaf_node_t &leaf = _leaf_node[i];

		//cout<<"leaf.doc.size()="<<leaf.doc.size()<<endl;
		if (leaf.doc.size() < 1)
			continue;
		++leaf_cnt;
	}
	treeDBsaver.write((char*)&leaf_cnt, sizeof(int));

	float counter = 0.0;
	float gap = leaf_cnt / 50.0f;

	for (int i = 0; i<(int)_num_leaf; ++i)
	{
		const leaf_node_t &leaf = _leaf_node[i];
		if (leaf.doc.size() < 1)
			continue;
		treeDBsaver.write((char *)&i, sizeof(int));
		int docsize = (int)leaf.doc.size();
		treeDBsaver.write((char *)&docsize, sizeof(int));
		std::map<int, float>::const_iterator it;
		for (it = leaf.doc.begin(); it != leaf.doc.end(); ++it)
		{
			treeDBsaver.write((char *)&it->first, sizeof(int));
			treeDBsaver.write((char *)&it->second, sizeof(float));
		}

		counter++;
		if (counter >= gap)
		{
			printf("*");
			counter = counter - gap;
		}
	}

	printf("Complete save tree %06d DB\n", doc_size);
	treeDBsaver.close();
	return 1;
}
int Tree::load_tree_DB(std::string _path)
{
	for (unsigned int i = 0; i < _num_leaf; i++)
		_leaf_node[i].doc.clear();

	std::ifstream treeDBloader(_path.c_str(), std::ios_base::in | std::ios_base::binary);
	if (!treeDBloader.is_open())
	{
		printf("Cannot open TreeDB file!\n");
		return 0;
	}

	treeDBloader.read((char*)&doc_size, sizeof(int));
	treeDBloader.read((char*)&total_desc, sizeof(int));

	int leaf_cnt = 0;
	treeDBloader.read((char*)&leaf_cnt, sizeof(int));

	float counter = 0.0;
	float gap = leaf_cnt / 50.0f;

	for (int i = 0; i<leaf_cnt; ++i)
	{
		int leafidx = 0;
		treeDBloader.read((char*)&leafidx, sizeof(int));
		leaf_node_t &leaf = _leaf_node[leafidx];
		int numdoc = 0;
		treeDBloader.read((char*)&numdoc, sizeof(int));

		for (int j = 0; j<numdoc; ++j)
		{
			int docid = -1; float score = 0.0f;
			treeDBloader.read((char*)&docid, sizeof(int));
			treeDBloader.read((char*)&score, sizeof(float));
			leaf.doc.insert(std::pair<int, float>(docid, score));
		}

		counter++;
		if (counter >= gap)
		{
			printf("*");
			counter = counter - gap;
		}

	}

	printf("Complete load tree %06d DB\n", doc_size);
	treeDBloader.close();
	return 1;
}
float Tree::dist_func(const float *f0, const float *f1)
{
	float r = 1.0f;
	for (int i = 0; i<pram.DESC_DIM; ++i)
		r -= (f0[i] * f1[i]);

	return r;
}
size_t Tree::find_leaf(cv::Mat &_disc)
{
	size_t idx = 0;  // root node
#pragma omp parallel for
	for (int lvl = 0; lvl < pram.LEVEL; ++lvl)
	{
		int_node_t &node = _int_node[idx];
		float dist, mindist = dist_func(node.c, (float *)_disc.data);
		int minidx = 0;
		for (int i = 1; i < pram.BRANCH_FACTOR; ++i)
			if ((dist = dist_func(&node.c[pram.DESC_DIM*i], (float *)_disc.data)) < mindist)
				mindist = dist, minidx = i;

		idx = child_idx(idx, minidx);
	}
	return idx;
}
bool Tree::insert_doc(int doc_id, cv::Mat &disc)
{
	doc_size++;
	total_desc += (int)disc.rows;
	int featcnt = (int)disc.rows;
	if (featcnt < 5) return 0;
	//for (size_t i = 0; i<(int)disc.rows; ++i)
	//	if (feat[i] != NULL)
	//		++featcnt;
	//if (featcnt <= 0) return false;

	cv::Mat ori_disc;
	if (disc.type() != 5)
	{
		ori_disc = disc.clone();
		disc.convertTo(disc, CV_32FC1);
	}

	const float w = 1.0f / (float)featcnt;
#pragma omp parallel for
	for (int i = 0; i<featcnt; ++i) {
		//if (feat[i] == NULL)
		//	continue;
		cv::Mat tmp = disc.row(i).clone();
		size_t idx = find_leaf(tmp);
		leaf_node_t &leaf = _leaf_node[leaf_idx(idx)];
		std::map<int, float>::iterator it = leaf.doc.find(doc_id);
		if (it == leaf.doc.end())
			leaf.doc.insert(std::pair<int, float>(doc_id, w));
		else {
			it->second += w;
		}
	}
	//doc_size++;
	//cout<<"doc_size"<<doc_size<<endl;

	if (!ori_disc.empty()) disc = ori_disc.clone();
	return 1;
}
void Tree::query_doc(cv::Mat &_disc, float *score_, float *likeli_)
{
	int numavgwords = (int)((float)total_desc / std::max((float)doc_size, 1.0f));

	//score_ = cv::Mat(1, doc_size, CV_32FC1, cv::Scalar(0));
	//likeli_ = cv::Mat(1, doc_size, CV_32FC1, cv::Scalar(1));

	int temp_doc_size = doc_size;

	if (pram.use_virtual_doc)
	{
		bool virtualdocremoved = false;
		std::vector<std::pair<int, int> > wordrptcnt;
#pragma omp parallel for
		for (int i = 0; i<_num_leaf; i++)
		{
			leaf_node_t &leaf = _leaf_node[i];
			if (leaf.doc.size() == 0)
				continue;

			std::map<int, float>::iterator jt;
			if ((jt = leaf.doc.find(-1)) != leaf.doc.end())
			{
				leaf.doc.erase(jt);
				virtualdocremoved = true;
			}
			wordrptcnt.push_back(std::pair<int, int>(-1 * leaf.doc.size(), i));
		}
		//if (virtualdocremoved == true)
		//	doc_size--;
#pragma omp parallel for
		if (wordrptcnt.size() > numavgwords)
		{
			std::sort(wordrptcnt.begin(), wordrptcnt.end());
			std::vector<std::pair<int, int> >::iterator it = wordrptcnt.begin();
			const float w2 = 1.0f / numavgwords;
			for (int i = 0; i<numavgwords; i++)
			{
				int rptwordidx = it->second;
				it++;
				leaf_node_t &leaf = _leaf_node[rptwordidx];
				std::map<int, float>::iterator jt = leaf.doc.find(-1);
				if (jt == leaf.doc.end()) {
					leaf.doc.insert(std::pair<int, float>(-1, w2));
				}
				else {
					jt->second += w2;
				}
			}
			//doc_size++;
			temp_doc_size++;
		}
	}


	std::map<size_t, int> q;
	int featcnt = 0;
#pragma omp parallel for
	for (int i = 0; i<(int)_disc.rows; ++i)
	{
		//if (feat[i] == NULL)
		//	continue;
		cv::Mat tmp = _disc.row(i).clone();
		size_t idx = find_leaf(tmp);
		const leaf_node_t &leaf = _leaf_node[leaf_idx(idx)];
		if (leaf.doc.size() > 0)
		{
			std::map<size_t, int>::iterator it = q.find(idx);
			if (it == q.end()) q.insert(std::pair<size_t, int>(idx, 1));
			else ++it->second;
		}
		++featcnt;
	}
	std::map<size_t, int>::const_iterator q_it;
	std::map<int, float> score;  // doc_id, score

	float sum = 0.0;
	float scoresqsum = 0.0;
	float mean = 0.0;
	float stddev = 0.0;
	int q_res_cnt = 1;
#pragma omp parallel for
	for (q_it = q.begin(); q_it != q.end(); ++q_it)
	{
		const leaf_node_t &leaf = _leaf_node[leaf_idx(q_it->first)];
		float n = q_it->second / (float)featcnt;  // query_doc's feat count
		n *= log((float)(temp_doc_size) / (float)leaf.doc.size());

		std::map<int, float>::const_iterator d_it;
		std::map<int, float>::iterator j;
		for (d_it = leaf.doc.begin(); d_it != leaf.doc.end(); ++d_it)
		{
			int doc_id = d_it->first;
			float m = d_it->second*log((float)(temp_doc_size) / (float)leaf.doc.size());
			float l1dist = -1.0f*(fabs(n - m) - n - m);

			if ((j = score.find(d_it->first)) == score.end())
			{
				score.insert(std::pair<int, float>(doc_id, l1dist));
				sum += l1dist;
				q_res_cnt++;
			}
			else
			{
				j->second += l1dist;
				sum += l1dist;
			}
		}
	}
	mean = sum / (float)q_res_cnt;
	sum = 0.0f;

	for (int i = 0; i<doc_size; i++)
	{
		std::map<int, float>::iterator j = score.find(i);
		if (j == score.end()) {
			score.insert(std::pair<int, float>(i, mean));
		}
	}

	for (std::map<int, float>::iterator j = score.begin(); j != score.end(); ++j)
	{
		scoresqsum += j->second * j->second;
		sum += j->second;
	}

	scoresqsum /= (float)score.size();
	mean = sum / (float)score.size();
	stddev = scoresqsum - mean*mean;
	stddev = sqrt(stddev);


	for (std::map<int, float>::iterator j = score.begin(); j != score.end(); ++j)
	{
		int idx = j->first;
		float val = j->second;

		if (idx == -1) continue;

		//score_.at<float>(0, idx) = val;
		score_[idx] = val;

		if (val > mean + pram.stddev_threshold*stddev)
		{
			likeli_[idx] = pram.likelihood_rate*((j->second - pram.stddev_threshold*stddev) / mean);
		}
		else
		{
			likeli_[idx] = 1.0f;
		}
	}
}
cv::Mat Tree::drawDistribution(cv::Mat *input, float nomalize)
{
	int height = 100;
	int col = (int)input->cols;
	int row = (int)input->rows;

	cv::Mat output(height * row + 10, std::max(col + 10, height * 2), CV_8UC3, cv::Scalar(255, 255, 255));

	if (col == 0 || row == 0)
		return output;

	float *max_val = new float[row];
	std::memset(max_val, 0, sizeof(float)*row);
	float *min_val = new float[row];
	std::memset(min_val, 0, sizeof(float)*row);
	float *normal = new float[row];//  1 all positive, 0 both, -1 all negative
	std::memset(normal, 0, sizeof(float)*row);
	float *base_height = new float[row];
	std::memset(base_height, 0, sizeof(float)*row);

	for (int col_i = 0; col_i < col; col_i++)
		for (int row_i = 0; row_i < row; row_i++)
		{
			if (input->at<float>(row_i, col_i) > max_val[row_i])
			{
				max_val[row_i] = input->at<float>(row_i, col_i);
			}
			if (input->at<float>(row_i, col_i) < min_val[row_i])
			{
				min_val[row_i] = input->at<float>(row_i, col_i);
			}
		}

	for (int row_i = 0; row_i < row; row_i++)
	{
		if (max_val[row_i] == 0 && min_val[row_i] == 0)
		{
			normal[row_i] = 1.0f;
			base_height[row_i] = (float)((row_i + 1)*height);
		}
		else if (max_val[row_i] == 0)
		{
			normal[row_i] = abs(min_val[row_i]);
			base_height[row_i] = (float)((row_i)*height) + 0.1f*(float)height;
		}
		else if (min_val[row_i] == 0)
		{
			normal[row_i] = abs(max_val[row_i]);;
			base_height[row_i] = (float)((row_i + 1)*height);
		}
		else
		{
			normal[row_i] = std::max(abs(max_val[row_i]), abs(min_val[row_i]));
			base_height[row_i] = (float)((row_i)*height) + 0.5f*(float)height;
		}
	}

	if (nomalize != 0)
	{
		for (int row_i = 0; row_i < row; row_i++)
		{
			normal[row_i] = abs(nomalize);
		}
	}

	for (int col_i = 0; col_i < col; col_i++)
		for (int row_i = 0; row_i < row; row_i++)
		{
			cv::Point base = cv::Point(col_i, (int)base_height[row_i]);
			cv::Point top = cv::Point(col_i, (int)(base_height[row_i] - 0.9f*(float)height * (input->at<float>(row_i, col_i) / normal[row_i])));
			cv::line(output, base, top, cv::Scalar(0, 0, 0));
		}

	return output;

	delete[] max_val;
	delete[] min_val;
	delete[] normal;
	delete[] base_height;
}

class LineObservation
{
private:
	struct SEGMENT
	{
		float x1, y1, x2, y2, angle;
		//cv::Mat msld;
		//std::vector<cv::Mat> desc;

		//float x1p, y1p, x2p, y2p, anglep;		
		//int label;
		//cv::Mat msld, np1;
		//std::vector<cv::Mat> desc;
		//size_t leafidx;

		//int label, frequency;
		//vector<Vector3d> epilines;
	};
	struct Pram
	{
		int threshold_length = 30;// "length threshold in line detection");
		float threshold_dist = 1.5f;
		int K = 50, L = 3, D = 72;
		std::string tree_path;
	};
	Pram pram;

	cv::Mat RGB;
	cv::Mat gray_RGB;

	void point_inboard_test(cv::Mat & src, cv::Point2i * pt);
	void getAngle(SEGMENT *seg);
	void AdditionalOperationsOnSegments(cv::Mat &gray_img, SEGMENT *seg);
	void incident_point(cv::Point2f * pt, cv::Mat & l);
	double dist_point_line(const cv::Mat & p, cv::Mat & l);
	void extract_segments(std::vector<cv::Point2i> * points, std::vector<SEGMENT> * segments);
	bool get_point_chain(const cv::Mat & img, const cv::Point pt, cv::Point * chained_pt, int & direction, int step);
	void line_detect(std::vector<SEGMENT> &output_line_);

	int Gx(cv::Mat & patch, int x, int y);
	int Gy(cv::Mat & patch, int x, int y);
	int bound_check(int x, int y, int w, int h);
	void BackwardWarping(cv::Mat & dst, cv::Mat &p_mat);
	void CalcMSLD(cv::Mat & patch, cv::Mat & msld);
	void GetImagePatch(cv::Mat &_output_patch, SEGMENT &seg);
	void line_compute(std::vector<SEGMENT> &_lines, cv::Mat &_disc);
	void draw_arrow(cv::Mat &output_, const SEGMENT &seg, cv::Scalar bgr);
public:
	Tree *line_tree;

	ObservationData *data;

	//struct Data
	//{
	//	//////////////////////////// input ////////////////////////////
	//	int &mode; // 0:localize, 1:save, 2:saveDB, 3:loadDB
	//	bool &draw_flag;
	//	cv::Mat RGB;
	//
	//	//////////////////////////// output ////////////////////////////
	//	int num_detected_feature = 0;
	//	cv::Mat score;
	//	cv::Mat likelihood;
	//};
	//Data data;

	LineObservation();
	~LineObservation();

	int run(void);
};
LineObservation::LineObservation()
{
	char temp_tree_path[256];
	sprintf(temp_tree_path, "/home/jswon/catkin_ws/src/virtual_fence/src/data/tree/msld_from_video2.bin_k%d_l%d.tree.cuda_rand", pram.K, pram.L);
	//sprintf_s(temp_tree_path, "..\\data\\tree\\msld_from_video2.bin_k%d_l%d.tree.cuda_rand", pram.K, pram.L);
	//std::cout << temp_tree_path << std::endl;
	pram.tree_path = std::string(temp_tree_path);

	line_tree = new Tree(50, 3, 72);
	line_tree->load_tree_structure(pram.tree_path);
}
LineObservation::~LineObservation()
{
	delete line_tree;
}
int LineObservation::Gx(cv::Mat &patch, int x, int y)
{
	return (patch.at<unsigned char>(y + 1, x - 1) + 2 * patch.at<unsigned char>(y + 1, x) + patch.at<unsigned char>(y + 1, x + 1))
		- (patch.at<unsigned char>(y - 1, x - 1) + 2 * patch.at<unsigned char>(y - 1, x) + patch.at<unsigned char>(y - 1, x + 1));
}
int LineObservation::Gy(cv::Mat &patch, int x, int y)
{
	return (patch.at<unsigned char>(y - 1, x + 1) + 2 * patch.at<unsigned char>(y, x + 1) +
		patch.at<unsigned char>(y + 1, x + 1)) - (patch.at<unsigned char>(y - 1, x - 1) +
			2 * patch.at<unsigned char>(y, x - 1) + patch.at<unsigned char>(y + 1, x - 1));
}
int LineObservation::bound_check(int x, int y, int w, int h)
{
	if (x<0 || x >= w || y<0 || y >= h) return 0;
	else return 1;
}
void LineObservation::BackwardWarping(cv::Mat & dst, cv::Mat &p_mat)
{
	double w, pixel, ratio, px, py;

	double wx[2];
	double wy[2];

	int i, j, x, y;

	cv::Mat rot_inv = p_mat.inv();

	for (j = 0; j < dst.rows; j++)
	{
		for (i = 0; i < dst.cols; i++)
		{
			ratio = pixel = 0.0;
			w = rot_inv.at<double>(2, 0)*i + rot_inv.at<double>(2, 1)*j + rot_inv.at<double>(2, 2);

			px = rot_inv.at<double>(0, 0)*i + rot_inv.at<double>(0, 1)*j + rot_inv.at<double>(0, 2);
			py = rot_inv.at<double>(1, 0)*i + rot_inv.at<double>(1, 1)*j + rot_inv.at<double>(1, 2);

			wx[1] = px - floor(px);
			wx[0] = 1.0 - wx[1];

			wy[1] = py - floor(py);
			wy[0] = 1.0 - wy[1];

			x = (int)floor(px);
			y = (int)floor(py);

			if (bound_check(x, y, gray_RGB.cols, gray_RGB.rows)) {
				pixel += wx[0] * wy[0] * gray_RGB.at<unsigned char>(y, x);
				ratio += wx[0] * wy[0];
			}
			if (bound_check(x + 1, y, gray_RGB.cols, gray_RGB.rows)) {
				pixel += wx[1] * wy[0] * gray_RGB.at<unsigned char>(y, x + 1);
				ratio += wx[1] * wy[0];
			}
			if (bound_check(x, y + 1, gray_RGB.cols, gray_RGB.rows)) {
				pixel += wx[0] * wy[1] * gray_RGB.at<unsigned char>(y + 1, x);
				ratio += wx[0] * wy[1];
			}
			if (bound_check(x + 1, y + 1, gray_RGB.cols, gray_RGB.rows)) {
				pixel += wx[1] * wy[1] * gray_RGB.at<unsigned char>(y + 1, x + 1);
				ratio += wx[1] * wy[1];
			}
			dst.at<unsigned char>(j, i) = (unsigned char)floor(pixel / ratio + 0.5);
		}
	}
}
void LineObservation::CalcMSLD(cv::Mat & patch, cv::Mat & msld)
{
	cv::Mat GDM = cv::Mat::zeros(9, patch.cols - 6, CV_64FC4);
	cv::Mat G = cv::Mat::zeros(5 * 9, patch.cols - 2, CV_64FC2); // 위아래 그래이 값 차이를 가우스 분산모양의 w 를 주어 그 위치에 저장
																 //	cout << GDM.rows << " " << GDM.cols << endl;
	double sigma = 22.5f;
	double d_sq_sigma = 2.0*(sigma*sigma);
	double gaussian_n = 1.0 / (sqrt(2.0*CV_PI)*sigma);
#pragma omp parallel for
	for (int i = 1; i<patch.rows - 1; i++)
	{
		for (int j = 1; j<patch.cols - 1; j++)
		{
			double gx = (double)Gx(patch, j, i); // j,i 포인트 기준 위아래 그래이값의 차이
			double gy = (double)Gy(patch, j, i); // j,i 포인트 기준 좌우 그래이값의 차이
			int d = abs(i - 23); //패치에서 선이 위치하는 중앙

			double w = gaussian_n * exp(-1.0*(double)(d*d) / d_sq_sigma);

			gx = w*gx;
			gy = w*gy;

			G.at<cv::Vec2d>(i - 1, j - 1)[0] = gx;
			G.at<cv::Vec2d>(i - 1, j - 1)[1] = gy;
		}
	}

#pragma omp parallel for
	for (int i = 0; i<GDM.rows; i++)
	{
		for (int j = 0; j<GDM.cols; j++)
		{
			double gx = 0.0, gy = 0.0;
			double w1 = 0.0, w2 = 0.0;
			int d1 = 0, d2 = 0;

			for (int k = 0; k<5; k++)
			{
				d1 = abs(2 - k);
				d2 = (3 + k) % 5;
				w1 = (double)d2 / (double)(d1 + d2);
				w2 = (double)d1 / (double)(d1 + d2);

				for (int l = 0; l<5; l++)
				{
					gx = G.at<cv::Vec2d>(k + 5 * i, l + j)[0];
					gy = G.at<cv::Vec2d>(k + 5 * i, l + j)[1];

					if (k <= 1 && gy >= 0.0 && i != 0)
					{
						GDM.at<cv::Vec4d>(i - 1, j)[0] += w2*gy;
						GDM.at<cv::Vec4d>(i, j)[0] += w1*gy;
					}
					else if (k <= 1 && gy<0.0 && i != 0)
					{
						GDM.at<cv::Vec4d>(i - 1, j)[1] += -w2*gy;
						GDM.at<cv::Vec4d>(i, j)[1] += -w1*gy;
					}

					if (k <= 1 && gx >= 0.0 && i != 0)
					{
						GDM.at<cv::Vec4d>(i - 1, j)[2] += w2*gx;
						GDM.at<cv::Vec4d>(i, j)[2] += w1*gx;
					}
					else if (k <= 1 && gx<0.0 && i != 0)
					{
						GDM.at<cv::Vec4d>(i - 1, j)[3] += -w2*gx;
						GDM.at<cv::Vec4d>(i, j)[3] += -w1*gx;
					}

					if (k == 2 && gy >= 0.0) {
						GDM.at<cv::Vec4d>(i, j)[0] += gy;
					}
					else if (k == 2 && gy<0.0) {
						GDM.at<cv::Vec4d>(i, j)[1] += -gy;
					}
					if (k == 2 && gx >= 0.0) {
						GDM.at<cv::Vec4d>(i, j)[2] += gx;
					}
					else if (k == 2 && gx<0.0) {
						GDM.at<cv::Vec4d>(i, j)[3] += -gx;
					}

					if (k >= 3 && gy >= 0.0 && i != GDM.rows - 1) {
						GDM.at<cv::Vec4d>(i + 1, j)[0] += w2*gy;
						GDM.at<cv::Vec4d>(i, j)[0] += w1*gy;
					}
					else if (k >= 3 && gy<0.0 && i != GDM.rows - 1) {
						GDM.at<cv::Vec4d>(i + 1, j)[1] += -w2*gy;
						GDM.at<cv::Vec4d>(i, j)[1] += -w1*gy;
					}

					if (k >= 3 && gx >= 0.0 && i != GDM.rows - 1)
					{
						GDM.at<cv::Vec4d>(i + 1, j)[2] += w2*gx;
						GDM.at<cv::Vec4d>(i, j)[2] += w1*gx;
					}
					else if (k >= 3 && gx<0.0 && i != GDM.rows - 1)
					{
						GDM.at<cv::Vec4d>(i + 1, j)[3] += -w2*gx;
						GDM.at<cv::Vec4d>(i, j)[3] += -w1*gx;
					}
					//else if(k>=3 && gx>=0.0 && i==GDM.rows-1)
					//{
					//	GDM.at<Vec4d>(i,j)[2] += gx;
					//}
					//else if(k>=3 && gx<0.0 && i==GDM.rows-1)
					//{
					//	GDM.at<Vec4d>(i,j)[3] += -gx;
					//}
				}
			}
			//cout << iv1 << "\t" << iv2 << "\t" << iv3 << "\t" << iv4 << endl;
			//GDM.at<Vec4d>(i,j)[0] += iv1;
			//GDM.at<Vec4d>(i,j)[1] += iv2;
			//GDM.at<Vec4d>(i,j)[2] += iv3;
			//GDM.at<Vec4d>(i,j)[3] += iv4;
		}
	}
	//	print_matrix(GDM);

	cv::Mat mean = cv::Mat::zeros(4 * 9, 1, CV_64FC1);
	cv::Mat stddev = cv::Mat::zeros(4 * 9, 1, CV_64FC1);
	double mag_mean = 0.0, mag_stddev = 0.0;

	for (int i = 0; i<GDM.cols; i++) {
		for (int j = 0; j<GDM.rows; j++) {
			for (int k = 0; k<4; k++) {
				mean.at<double>(j * 4 + k, 0) += GDM.at<cv::Vec4d>(j, i)[k];
			}
		}
	}
	for (int i = 0; i<mean.rows; i++) {
		mean.at<double>(i, 0) /= (double)GDM.cols;
		mag_mean += mean.at<double>(i, 0)*mean.at<double>(i, 0);
	}
	mag_mean = sqrt(mag_mean);
	//	cout << "mag_mean: " << mag_mean << endl;

	for (int i = 0; i<GDM.cols; i++) {
		for (int j = 0; j<GDM.rows; j++) {
			for (int k = 0; k<4; k++) {
				stddev.at<double>(j * 4 + k, 0) +=
					(GDM.at<cv::Vec4d>(j, i)[k] - mean.at<double>(j * 4 + k, 0))*
					(GDM.at<cv::Vec4d>(j, i)[k] - mean.at<double>(j * 4 + k, 0));
			}
		}
	}

	for (int i = 0; i<stddev.rows; i++) {
		stddev.at<double>(i, 0) /= (double)GDM.cols;
		stddev.at<double>(i, 0) = sqrt(stddev.at<double>(i, 0));
		mag_stddev += stddev.at<double>(i, 0)*stddev.at<double>(i, 0);
	}
	mag_stddev = sqrt(mag_stddev);
	//	cout << "mag_stddev: " << mag_stddev << endl;
	//	print_matrix(stddev);

	for (int i = 0; i<8 * 9; i++)
	{
		if (i<8 * 9 / 2)
			msld.at<float>(0, i) = (float)(mean.at<double>(i, 0) / mag_mean) / 1.414213562f;
		else
			msld.at<float>(0, i) = (float)(stddev.at<double>(i - 8 * 9 / 2, 0) / mag_stddev) / 1.414213562f;
	}
	//	print_matrix(stddev);
	//	print_matrix(msld);
}
void LineObservation::GetImagePatch(cv::Mat &_output_patch, SEGMENT &seg)
{
	cv::Mat rot = cv::Mat::zeros(3, 3, CV_64FC1);

	double offset_x = 0.0;
	double offset_y = 0.0;

	cv::Mat l = cv::Mat::zeros(3, 1, CV_64FC1);
	l.at<double>(0, 0) = (double)(seg.x2 - seg.x1);
	l.at<double>(1, 0) = (double)(seg.y2 - seg.y1);
	l.at<double>(2, 0) = 1.0;


	double s = sqrt(l.at<double>(0, 0) * l.at<double>(0, 0) + l.at<double>(1, 0) *
		l.at<double>(1, 0));

	cv::Mat ln = cv::Mat::zeros(3, 1, CV_64FC1);
	ln.at<double>(0, 0) = l.at<double>(0, 0) / s;
	ln.at<double>(1, 0) = l.at<double>(1, 0) / s;
	ln.at<double>(2, 0) = 1.0;


	cv::Mat n = cv::Mat::zeros(3, 1, CV_64FC1);
	n.at<double>(0, 0) = (double)(cos(90.0*CV_PI / 180.0)*l.at<double>(0, 0) -
		sin(90.0*CV_PI / 180.0)*l.at<double>(1, 0));
	n.at<double>(1, 0) =
		(double)(sin(90.0*CV_PI / 180.0)*l.at<double>(0, 0) +
			cos(90.0*CV_PI / 180.0)*l.at<double>(1, 0));
	n.at<double>(2, 0) = 1.0;


	s = sqrt(n.at<double>(0, 0)*n.at<double>(0, 0) + n.at<double>(1, 0)
		*n.at<double>(1, 0));

	n.at<double>(0, 0) = n.at<double>(0, 0) / s;
	n.at<double>(1, 0) = n.at<double>(1, 0) / s;


	cv::Mat e1 = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat e2 = cv::Mat::zeros(3, 1, CV_64FC1);
	e1.at<double>(0, 0) = (double)seg.x1;
	e1.at<double>(1, 0) = (double)seg.y1;
	e1.at<double>(2, 0) = 0.0;
	e2.at<double>(0, 0) = (double)seg.x2;
	e2.at<double>(1, 0) = (double)seg.y2;
	e2.at<double>(2, 0) = 0.0;



	rot.at<double>(0, 0) = cos(-seg.angle);
	rot.at<double>(0, 1) = -sin(-seg.angle);
	rot.at<double>(1, 0) = sin(-seg.angle);
	rot.at<double>(1, 1) = cos(-seg.angle);
	rot.at<double>(2, 2) = 1.0;
	rot.at<double>(0, 2) = 0.0;
	rot.at<double>(1, 2) = 0.0;


	cv::Mat v1 = 23.0*n + e1 - 3.0*ln;
	v1 = rot * v1;
	cv::Mat v2 = -23.0*n + e1 - 3.0*ln;
	v2 = rot * v2;
	cv::Mat v3 = 23.0*n + e2 + 3.0*ln;
	v3 = rot * v3;
	cv::Mat v4 = -23.0*n + e2 + 3.0*ln;
	v4 = rot * v4;

	offset_x = v1.at<double>(0, 0) < v2.at<double>(0, 0) ? v1.at<double>(0, 0) : v2.at<double>(0, 0);
	offset_x = offset_x < v3.at<double>(0, 0) ? offset_x : v3.at<double>(0, 0);
	offset_x = offset_x < v4.at<double>(0, 0) ? offset_x : v4.at<double>(0, 0);

	offset_y = v1.at<double>(1, 0) < v2.at<double>(1, 0) ? v1.at<double>(1, 0) : v2.at<double>(1, 0);
	offset_y = offset_y < v3.at<double>(1, 0) ? offset_y : v3.at<double>(1, 0);
	offset_y = offset_y < v4.at<double>(1, 0) ? offset_y : v4.at<double>(1, 0);

	rot.at<double>(0, 0) = cos(-seg.angle);
	rot.at<double>(0, 1) = -sin(-seg.angle);
	rot.at<double>(1, 0) = sin(-seg.angle);
	rot.at<double>(1, 1) = cos(-seg.angle);
	rot.at<double>(2, 2) = 1.0;
	rot.at<double>(0, 2) = -offset_x;
	rot.at<double>(1, 2) = -offset_y;

	BackwardWarping(_output_patch, rot);
}
void LineObservation::line_compute(std::vector<SEGMENT> &_lines, cv::Mat &_disc)
{
#pragma omp parallel for
	for (int i = 0; i < (int)_lines.size(); i++)
	{
		SEGMENT seg = _lines.at(i);
		//cv::Mat msld = cv::Mat::zeros(pram.D, 1, CV_32FC1);
		double length = sqrt((seg.x1 - seg.x2)*(seg.x1 - seg.x2) + (seg.y1 - seg.y2)*(seg.y1 - seg.y2));
		cv::Mat patch = cv::Mat::zeros(45 + 2, cvRound(length) + 6, CV_8UC1);

		GetImagePatch(patch, seg);
		cv::Mat tmp = _disc.row(i).clone();
		CalcMSLD(patch, tmp);

		//_lines.at(i).msld = msld.clone();
		//_lines.at(i).desc.push_back(msld.t());
	}
}
void LineObservation::point_inboard_test(cv::Mat & src, cv::Point2i * pt)
{
	pt->x = pt->x <= 5 ? 5 : pt->x >= src.cols - 5 ? src.cols - 5 : pt->x;
	pt->y = pt->y <= 5 ? 5 : pt->y >= src.rows - 5 ? src.rows - 5 : pt->y;
}
void LineObservation::getAngle(SEGMENT *seg)
{
	float fDx = (float)(seg->x2 - seg->x1);
	float fDy = (float)(seg->y2 - seg->y1);
	float fTemp = 0.0f;
	double dAngle = 0.0;

	if (fDx == 0.0f) {
		if (fDy > 0)
			dAngle = CV_PI / 2.0;
		else
			dAngle = -1.0 * CV_PI / 2.0;
	}
	else if (fDy == 0.0f) {
		if (fDx > 0)
			dAngle = 0.0;
		else
			dAngle = CV_PI;
	}
	else if (fDx < 0.0f && fDy > 0.0f)
		dAngle = CV_PI + atan(fDy / fDx);
	else if (fDx > 0.0f && fDy < 0.0f)
		dAngle = 2 * CV_PI + atan(fDy / fDx);
	else if (fDx < 0.0f && fDy < 0.0f)
		dAngle = CV_PI + atan(fDy / fDx);
	else
		dAngle = atan(fDy / fDx);

	if (dAngle > 2.0 * CV_PI)
		dAngle -= 2.0 * CV_PI;
	seg->angle = (float)dAngle;
}
void LineObservation::AdditionalOperationsOnSegments(cv::Mat &gray_img, SEGMENT * seg)
{
	if (seg->x1 == 0.0f && seg->x2 == 0.0f && seg->y1 == 0.0f && seg->y2 == 0.0f) return;

	float fDx, fDy, fTemp;

	//	double dAngle;
	//	fDx = (float)(seg->x2 - seg->x1);
	//	fDy = (float)(seg->y2 - seg->y1);
	//	fTemp = 0.0f;
	//	if(fDx == 0.0f) {
	//		if(fDy > 0)
	//			dAngle = CV_PI / 2.0;
	//		else
	//			dAngle = -1.0 * CV_PI / 2.0;
	//	}
	//	else if(fDy == 0.0f) {
	//		if(fDx > 0)
	//			dAngle = 0.0;
	//		else
	//			dAngle = CV_PI;
	//	}
	//	else if(fDx < 0.0f && fDy > 0.0f)
	//		dAngle = CV_PI + atan( fDy/fDx );
	//	else if(fDx > 0.0f && fDy < 0.0f)
	//		dAngle = 2*CV_PI + atan( fDy/fDx );
	//	else if(fDx < 0.0f && fDy < 0.0f)
	//		dAngle = CV_PI + atan( fDy/fDx );
	//	else
	//		dAngle = atan( fDy/fDx );
	//	if(dAngle > 2.0 * CV_PI)
	//		dAngle -= 2.0 * CV_PI;
	//	seg->angle = (float)dAngle;

	getAngle(seg);
	double dAngle = (double)seg->angle;

	cv::Point2f pStart = cv::Point2f(seg->x1, seg->y1);
	cv::Point2f pEnd = cv::Point2f(seg->x2, seg->y2);

	double dDx = 0.0, dDy = 0.0;
	dDx = (double)pEnd.x - (double)pStart.x;
	dDy = (double)pEnd.y - (double)pStart.y;

	int iNCP = 10;
	cv::Point2f *pCP = new cv::Point2f[iNCP];

	pCP[0] = pStart;
	pCP[iNCP - 1] = pEnd;
	for (int i = 0; i < iNCP; i++)
	{
		if (i == 0 || i == iNCP - 1)
		{
			continue;
		}
		pCP[i].x = pCP[0].x + ((float)dDx / (float)(iNCP - 1) * (float)i);
		pCP[i].y = pCP[0].y + ((float)dDy / (float)(iNCP - 1) * (float)i);
	}

	cv::Point2i *pCPR = new cv::Point2i[iNCP];
	cv::Point2i *pCPL = new cv::Point2i[iNCP];

	double dGap = 1.0;

	for (int i = 0; i < iNCP; i++)
	{
		pCPR[i].x = cvRound(pCP[i].x + dGap*cos(90.0 * CV_PI / 180.0 + dAngle));
		pCPR[i].y = cvRound(pCP[i].y + dGap*sin(90.0 * CV_PI / 180.0 + dAngle));
		pCPL[i].x = cvRound(pCP[i].x - dGap*cos(90.0 * CV_PI / 180.0 + dAngle));
		pCPL[i].y = cvRound(pCP[i].y - dGap*sin(90.0 * CV_PI / 180.0 + dAngle));
		point_inboard_test(gray_img, &pCPR[i]);
		point_inboard_test(gray_img, &pCPL[i]);
	}

	int iR = 0, iL = 0;
	for (int i = 0; i < iNCP; i++)
	{
		iR += gray_img.at<unsigned char>(pCPR[i].y, pCPR[i].x);
		iL += gray_img.at<unsigned char>(pCPL[i].y, pCPL[i].x);
	}

	if (iR > iL)
	{
		fTemp = seg->x1; seg->x1 = seg->x2; seg->x2 = fTemp;
		fTemp = seg->y1; seg->y1 = seg->y2; seg->y2 = fTemp;

		fDx = (float)(seg->x2 - seg->x1);
		fDy = (float)(seg->y2 - seg->y1);

		dAngle = dAngle + CV_PI;
		if (dAngle >= 2.0*CV_PI)
			dAngle = dAngle - 2.0 * CV_PI;
		seg->angle = (float)dAngle;
	}

	delete[] pCP; delete[] pCPR; delete[] pCPL;

	//seg->label = init_label++;
	return;
}
void LineObservation::incident_point(cv::Point2f * pt, cv::Mat & l)
{
	double a[] = { (double)pt->x, (double)pt->y, 1.0 };
	double b[] = { l.at<double>(0, 0), l.at<double>(1, 0), 0.0 };
	double c[3];

	cv::Mat xk = cv::Mat(3, 1, CV_64FC1, a).clone();
	cv::Mat lh = cv::Mat(3, 1, CV_64FC1, b).clone();
	cv::Mat lk = cv::Mat(3, 1, CV_64FC1, c).clone();

	lk = xk.cross(lh);
	xk = lk.cross(l);

	double s = 1.0 / xk.at<double>(2, 0);
	xk.convertTo(xk, -1, s);

	pt->x = ((float)xk.at<double>(0, 0) < 0.0f ? 0.0f : (float)xk.at<double>(0, 0) >= ((float)RGB.cols - 1.0f) ? ((float)RGB.cols - 1.0f) : (float)xk.at<double>(0, 0));
	pt->y = ((float)xk.at<double>(1, 0) < 0.0f ? 0.0f : (float)xk.at<double>(1, 0) >= ((float)RGB.rows - 1.0f) ? ((float)RGB.rows - 1.0f) : (float)xk.at<double>(1, 0));
}
double LineObservation::dist_point_line(const cv::Mat & p, cv::Mat & l)
{
	double x, y, w;

	x = l.at<double>(0, 0);
	y = l.at<double>(1, 0);

	w = sqrt(x*x + y*y);

	l.at<double>(0, 0) = x / w;
	l.at<double>(1, 0) = y / w;
	l.at<double>(2, 0) = l.at<double>(2, 0) / w;

	return l.dot(p);
}
void LineObservation::extract_segments(std::vector<cv::Point2i> * points, std::vector<SEGMENT> * segments)
{
	bool is_line;

	int i, j;
	SEGMENT seg;
	cv::Point2i ps, pe, pt;

	std::vector<cv::Point2i> l_points;

	int total = (int)(points->size());

	for (i = 0; i + pram.threshold_length < total; i++)
	{
		ps = points->at(i);
		pe = points->at(i + pram.threshold_length);

		double a[] = { (double)ps.x, (double)ps.y, 1 };
		double b[] = { (double)pe.x, (double)pe.y, 1 };
		double c[3], d[3];

		cv::Mat p1 = cv::Mat(3, 1, CV_64FC1, a).clone();
		cv::Mat p2 = cv::Mat(3, 1, CV_64FC1, b).clone();
		cv::Mat p = cv::Mat(3, 1, CV_64FC1, c).clone();
		cv::Mat l = cv::Mat(3, 1, CV_64FC1, d).clone();
		l = p1.cross(p2);

		is_line = true;

		//		while(!l_points.empty()) l_points.pop_back();
		l_points.clear();
		l_points.push_back(ps);

		for (j = 1; j < pram.threshold_length; j++)
		{
			pt.x = points->at(i + j).x;
			pt.y = points->at(i + j).y;

			p.at<double>(0, 0) = (double)pt.x;
			p.at<double>(1, 0) = (double)pt.y;
			p.at<double>(2, 0) = 1.0;

			double dist = dist_point_line(p, l);

			if (fabs(dist) > pram.threshold_dist)
			{
				is_line = false;
				break;
			}
			l_points.push_back(pt);
		}

		// Line check fail, test next point
		if (is_line == false)
		{
			continue;
		}
		l_points.push_back(pe);

		cv::Vec4f line;
		fitLine(cv::Mat(l_points), line, CV_DIST_L2, 0, 0.01, 0.01);
		a[0] = line[2];
		a[1] = line[3];
		b[0] = line[2] + line[0];
		b[1] = line[3] + line[1];

		p1 = cv::Mat(3, 1, CV_64FC1, a).clone();
		p2 = cv::Mat(3, 1, CV_64FC1, b).clone();

		l = p1.cross(p2);

		cv::Point2f ps_float((float)ps.x, (float)ps.y);
		incident_point(&ps_float, l);
		ps.x = (int)ps_float.x;
		ps.y = (int)ps_float.y;

		// Extending line
		for (j = pram.threshold_length + 1; i + j < total; j++)
		{
			pt.x = points->at(i + j).x;
			pt.y = points->at(i + j).y;

			p.at<double>(0, 0) = (double)pt.x;
			p.at<double>(1, 0) = (double)pt.y;
			p.at<double>(2, 0) = 1.0;

			double dist = dist_point_line(p, l);

			if (fabs(dist) > pram.threshold_dist)
			{
				j--;
				break;
			}

			pe = pt;
			l_points.push_back(pt);
		}
		fitLine(cv::Mat(l_points), line, CV_DIST_L2, 0, 0.01, 0.01);
		a[0] = line[2];
		a[1] = line[3];
		b[0] = line[2] + line[0];
		b[1] = line[3] + line[1];

		p1 = cv::Mat(3, 1, CV_64FC1, a).clone();
		p2 = cv::Mat(3, 1, CV_64FC1, b).clone();

		l = p1.cross(p2);

		cv::Point2f e1, e2;
		e1.x = (float)ps.x;
		e1.y = (float)ps.y;
		e2.x = (float)pe.x;
		e2.y = (float)pe.y;

		incident_point(&e1, l);
		incident_point(&e2, l);
		seg.x1 = e1.x;
		seg.y1 = e1.y;
		seg.x2 = e2.x;
		seg.y2 = e2.y;

		segments->push_back(seg);
		i = i + j;
	}
}
bool LineObservation::get_point_chain(const cv::Mat & img, const cv::Point pt, cv::Point * chained_pt, int & direction, int step)
{
	int ri, ci;
	int indices[8][2] = { { 1, 1 },{ 1, 0 },{ 1, -1 },{ 0, -1 },{ -1, -1 },{ -1, 0 },{ -1, 1 },{ 0, 1 } };

	for (int i = 0; i < 8; i++)
	{
		ci = pt.x + indices[i][1];
		ri = pt.y + indices[i][0];

		if (ri < 0 || ri == img.rows || ci < 0 || ci == img.cols)
			continue;

		if (img.at<unsigned char>(ri, ci) == 0)
			continue;

		if (step == 0)
		{
			chained_pt->x = ci;
			chained_pt->y = ri;
			direction = i;
			return true;
		}
		else
		{
			if (abs(i - direction) <= 2 || abs(i - direction) >= 6)
			{
				chained_pt->x = ci;
				chained_pt->y = ri;
				direction = i;
				return true;
			}
			else
				continue;
		}
	}
	return false;
}
void LineObservation::line_detect(std::vector<SEGMENT> &output_line_)
{
	if (output_line_.size() != 0) output_line_.clear();

	cv::Mat canny;
	cv::Canny(gray_RGB, canny, 50, 20, 3);
	//cv::imshow("canny", canny);

	for (int i = 0; i<canny.rows; i++)
		for (int j = 0; j<canny.cols; j++)
			if (i < 5 || i > canny.rows - 5 || j < 5 || j > gray_RGB.cols - 5)
				canny.at<unsigned char>(i, j) = 0;

	SEGMENT seg;
#pragma omp parallel for
	for (int r = 0; r < canny.rows; r++)
		for (int c = 0; c < canny.cols; c++)
		{
			// Find seeds - skip for non-seeds
			if (canny.at<unsigned char>(r, c) == 0)
				continue;

			std::vector<cv::Point2i> points;

			// Found seeds
			cv::Point2i pt(c, r);
			int direction = -1;
			int step = 0;
			while (get_point_chain(canny, pt, &pt, direction, step))
			{
				points.push_back(pt);
				step++;
				canny.at<unsigned char>(pt.y, pt.x) = 0;
			}

			if ((int)points.size() < pram.threshold_length) continue;

			std::vector<SEGMENT> segments;
			extract_segments(&points, &segments);


			for (int i = 0; i < (int)segments.size(); i++)
			{
				seg = segments.at(i);
				float fLength = sqrt((seg.x1 - seg.x2)*(seg.x1 - seg.x2) + (seg.y1 - seg.y2)*(seg.y1 - seg.y2));
				if (fLength < pram.threshold_length) continue;
				if ((seg.x1 <= 5.0f && seg.x2 <= 5.0f)
					|| (seg.y1 <= 5.0f && seg.y2 <= 5.0f)
					|| (seg.x1 >= (float)RGB.cols - 5.0f && seg.x2 >= (float)RGB.cols - 5.0f)
					|| (seg.y1 >= (float)RGB.rows - 5.0f && seg.y2 >= (float)RGB.rows - 5.0f)) continue;

				AdditionalOperationsOnSegments(gray_RGB, &seg);
				output_line_.push_back(seg);
			}
		}

	//bool is_merged = false;//, is_merged2 = false;
	//
	//while(segments_tmp.size() > 0)
	//{
	//
	//	if(segments_tmp.size() == 1)
	//	{
	//		seg1 = segments_tmp.back();
	//		segments_tmp.pop_back();
	//		segments_tmp2.push_back(seg1);
	//		break;
	//	}
	//	else
	//	{
	//		seg1 = segments_tmp.back();
	//		segments_tmp.pop_back();
	//		seg2 = segments_tmp.back();
	//		segments_tmp.pop_back();
	//
	//		is_merged = merge_segments(&seg1, &seg2, &seg2);
	//		if(is_merged == true)
	//		{
	//			AdditionalOperationsOnSegments(src, &seg2);
	//			segments_tmp.push_back(seg2);
	//		}
	//		else
	//		{
	//			segments_tmp.push_back(seg2);
	//			segments_tmp2.push_back(seg1);
	//		}
	//	}
	//}
	//
	//bool *IsAdded = new bool[segments_tmp2.size()];
	//memset(IsAdded, false, segments_tmp2.size());
	//
	//for(int i = 0; i < (int)segments_tmp2.size(); i++)
	//{
	//	seg1 = segments_tmp2.at(i);
	//	if(IsAdded[i] == true) continue;
	//
	//	is_merged = false;
	//	for(int j = 0; j < (int)segments_tmp2.size(); j++)
	//	{
	//		if(i == j || IsAdded[j] == true) continue;
	//
	//		seg2 = segments_tmp2.at(j);
	//
	//		is_merged = merge_segments(&seg1, &seg2, &seg2);
	//
	//		if(is_merged == true)
	//		{
	//			AdditionalOperationsOnSegments(src, &seg2);
	//			segments_all->push_back(seg2);
	//			LineDetector::draw_arrow(src, &seg2, Scalar(0, 0, 0));
	//			IsAdded[j] = true;
	//			IsAdded[i] = true;
	//		}
	//	}
	//	if(IsAdded[i] != true)
	//	{
	//		segments_all->push_back(seg1);
	//		LineDetector::draw_arrow(src, &seg1, Scalar(0, 0, 0));
	//		IsAdded[i] = true;
	//	}
	//}
	//delete[] IsAdded;
	//
	//imshow("edge2", src);
}
void LineObservation::draw_arrow(cv::Mat &output_, const SEGMENT &seg, cv::Scalar bgr)
{
	cv::Point2i p1;

	double dGap = 10.0;
	double dAngle = (double)seg.angle;
	double dArrowAng = 30.0;

	p1.x = cvRound(seg.x2 - dGap*cos(dArrowAng * CV_PI / 180.0 + dAngle));
	p1.y = cvRound(seg.y2 - dGap*sin(dArrowAng * CV_PI / 180.0 + dAngle));
	point_inboard_test(output_, &p1);

	line(output_, cv::Point(cvRound(seg.x1), cvRound(seg.y1)), cv::Point(cvRound(seg.x2), cvRound(seg.y2)), bgr, 1, 1);
	line(output_, cv::Point(cvRound(seg.x2), cvRound(seg.y2)), p1, bgr, 1, 1);
}
int LineObservation::run(void)
{
	if (data->mode == 2) //save
	{
		line_tree->save_tree_DB("..\\data\\node.dat");
		data->mode = 0;
	}

	if (data->mode == 3) //load
	{
		line_tree->load_tree_DB("..\\data\\node.dat");
		data->mode = 0;
	}


	if (data->row == 0 || data->col == 0)
	{
		return 1;
	}

	RGB = cv::Mat(data->row, data->col, CV_8UC3, cv::Scalar(0));
	std::memcpy(RGB.data, data->RGB_data, sizeof(char) * 3 * data->row * data->col);

	cv::cvtColor(RGB, gray_RGB, CV_RGB2GRAY);

	std::vector<SEGMENT> lines;
	line_detect(lines);

	cv::Mat descriptor((int)lines.size(), pram.D, CV_32FC1, cv::Scalar(0));
	line_compute(lines, descriptor);

	if (data->mode == 1)
	{
		bool inserting = line_tree->insert_doc(line_tree->doc_size, descriptor);
		if (!inserting)
			printf("There is no feature in %06dimg. Observation name = line\n", line_tree->doc_size - 1);

		data->mode = 0;
	}

	float *temp_likeli = new float[line_tree->doc_size];

	line_tree->query_doc(descriptor, data->score, temp_likeli);

	data->num_DB_size = line_tree->doc_size;
	//data->mtx.lock();
	std::memcpy(data->likelihood, temp_likeli, sizeof(float) * line_tree->doc_size);
	//data->mtx.unlock();

	delete[] temp_likeli;

	cv::Mat line_plot = RGB.clone();
	for (int i = 0; i < (int)lines.size(); i++)
		draw_arrow(line_plot, lines.at(i), cv::Scalar(0, 255, 0));

	data->line_plot = line_plot;

	if (data->draw_flag)
	{
		cv::Mat plot_score(1, line_tree->doc_size, CV_32FC1, data->score);
		cv::Mat plot_likelihood(1, line_tree->doc_size, CV_32FC1, data->likelihood);
		
		cv::imshow("line_plot", line_plot);
		cv::imshow("Line_score", line_tree->drawDistribution(&plot_score));
		cv::imshow("Line_likelihood", line_tree->drawDistribution(&plot_likelihood));
		cv::waitKey(10);
	}
	else
	{
		cv::destroyWindow("line_plot");
		cv::destroyWindow("Line_score");
		cv::destroyWindow("Line_likelihood");
	}

	if (data->mode == 9)
	{
		data->mode = 0;
		delete this;
		return 0;
	}

	return 1;
}