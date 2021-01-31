#include "ndt_registration/ndt_matcher_d2d_2d.h"
#include "ndt_map/ndt_cell.h"
#include "ndt_map/lazy_grid.h"
#include "ndt_map/pointcloud_utils.h"
#include "Eigen/Eigen"
#include <fstream>
#include <omp.h>
#include <time.h>
#include "time_patch.h"

int matchCycles = 0;
double runTime2 = 0;



namespace perception_oru
{
	void NDTMatcherD2D_2D::init(bool useDefaultGridResolutions, std::vector<double> _resolutions)
	{
		Jest.setZero();
		Jest.block<2, 2>(0, 0).setIdentity();
		Hest.setZero();
		Zest.setZero();
		ZHest.setZero();

		// 如果声明了的话，使用缺省分辨率(0.2 -> 0.5 -> 1.0 -> 2.0)
		if (useDefaultGridResolutions)
		{
			resolutions.push_back(0.2);
			resolutions.push_back(0.5);
			resolutions.push_back(1);
			resolutions.push_back(2);
		}
		// 否则采用提供的分辨率
		else
			resolutions = _resolutions;

		current_resolution = 0.1; // Argggg!!! This is very important to have initiated! (one day debugging later) :-) TODO do we need to set it to anything better?
		lfd1 = 1; //lfd1/(double)sourceNDT.getMyIndex()->size(); //current_resolution*2.5;
		lfd2 = 0.05; //0.1/current_resolution;
		ITR_MAX = 30;
		DELTA_SCORE = 10e-3 * current_resolution;
		n_neighbours = 2;
	}

	//
	// 对两个点云进行2D配准。
	// 其中，target作为参考点云，source是当前点云。T的输入值为变换的初始估测值，
	// 它的输出值为配准结果。
	//
	bool NDTMatcherD2D_2D::match(pcl::PointCloud<pcl::PointXYZ>& target,
		pcl::PointCloud<pcl::PointXYZ>& source,
		Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor>& T,
		bool useInitialGuess)
	{
		//initial guess
		pcl::PointCloud<pcl::PointXYZ> sourceCloud = source;

		Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> Temp, Tinit;
		Tinit.setIdentity();

		// 如果应用初始估测值
		if (useInitialGuess)
		{
			perception_oru::transformPointCloudInPlace(T, sourceCloud);
			Tinit = T;
		}

		T.setIdentity();
		bool ret = false;

		// 对于规则栅格，按照“由初到细”的分辨率，进行匹配迭代
		for (int r_ctr = resolutions.size() - 1; r_ctr >= 0; r_ctr--)
		{
			// 取下一个分辨率
			current_resolution = resolutions[r_ctr];

			// 分配栅格
			LazyGrid prototypeSource(current_resolution);
			LazyGrid prototypeTarget(current_resolution);

			// 重新生成参考点云的NDT图(此处是否可优化?)
			NDTMap targetNDT(&prototypeTarget);
			targetNDT.loadPointCloud(target);
			targetNDT.computeNDTCells();

			// 生成当前点云的NDT图
			NDTMap sourceNDT(&prototypeSource);
			sourceNDT.loadPointCloud(sourceCloud);
			sourceNDT.computeNDTCells();

			// 对两个NDT图进行匹配，得到坐标变换Temp
			Temp.setIdentity();
			int nCountGood = 0;
			int nSourceNDT = 0;
			ret = this->match(targetNDT, sourceNDT, Temp, nCountGood,
				nSourceNDT);

			// 对当前点云进行上述的坐标变换
			perception_oru::transformPointCloudInPlace(Temp, sourceCloud);

			// 累计坐标变换
			T = Temp * T;
		}

		if (useInitialGuess)
			T = T * Tinit;

		return ret;
	}

	void NDTMatcherD2D_2D::HandleSpecialCase(Eigen::MatrixXd& H, double regularizer)
	{
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 3, 3> > Sol(H);
		Eigen::Matrix<double, 3, 1> evals = Sol.eigenvalues().real();
		double minCoeff = evals.minCoeff();
		double maxCoeff = evals.maxCoeff();

		if (minCoeff < 10e-3)   //|| evals.minCoeff() < 0.001*evals.maxCoeff())
		{
			Eigen::Matrix<double, 3, 3> evecs = Sol.eigenvectors().real();
			regularizer = regularizer + minCoeff > 0 ? regularizer : 0.001*maxCoeff - minCoeff;

			//double regularizer = 0.001*maxCoeff - minCoeff;
			Eigen::Matrix<double, 3, 1> reg;

			//ugly
			reg << regularizer, regularizer, regularizer;
			evals += reg;
			Eigen::Matrix<double, 3, 3> Lam;
			Lam = evals.asDiagonal();
			H = evecs * Lam * (evecs.transpose());
		}
	}

	//
	// 对两个点NDT图进行2D配准。
	// 其中，targetNDT作为参考图，sourceNDT是当前图。T的输入值为变换的初始估测值，
	// 它的输出值为配准结果。
	//
	bool NDTMatcherD2D_2D::match(NDTMap& targetNDT,	NDTMap& sourceNDT,
		Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor>& T, int& nCountGood,
		int& nSourceNDT, bool useInitialGuess)
	{
		//locals
		bool convergence = false;
		int itr_ctr = 0;
		Eigen::Matrix<double, 3, 1>  pose_increment_v, scg;

		Eigen::MatrixXd H(3, 3), score_gradient_2d(3, 1);
		Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> TR, Tbest;

		bool ret = true;
		double score_best = INT_MAX;
		finalscore = 0;
		
		runTime = 0;
		runTime2 = 0;

		if (!useInitialGuess)
			T.setIdentity();

		Tbest = T;

		double t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0, t = 0;

		double d1 = getDoubleTime();

		// 将当前NDT图进行变换，并以一组NDT单元向量的形式返回
		nextNDT.Clear();
		nextNDT0.Clear();
		bMatchOK = false;

		nextNDT = sourceNDT.pseudoTransformNDT(T);
		nextNDT0 = sourceNDT.pseudoTransformNDT(T);

		t1 = getDoubleTime() - d1;
		runTime2 += t1;

		matchCycles = 0;

		// 循环直到收敛
		while (!convergence)
		{
			H.setZero();
			score_gradient_2d.setZero();

			double d2 = getDoubleTime();
		
			// 取得当前得分
			double score_here = derivativesNDT_2d(nextNDT, targetNDT, score_gradient_2d, H, true);
			
			t2 += getDoubleTime() - d2;
			runTime2 += getDoubleTime() - d2;

			scg = score_gradient_2d;

			// 记录“最佳得分”(最小者)和“最佳变换”
			if (score_here < score_best)
			{
				Tbest = T;
				score_best = score_here;
			}

			double gr_norm = score_gradient_2d.norm();

			// 如果梯度几乎不再变化(梯度得分矩阵的模很小)
			if (score_gradient_2d.norm() <= 10e-2 * DELTA_SCORE)
			{
				// 如果当前得分不如“最佳得分”，T取“最佳变换”Tbest
				if (score_here > score_best)
					T = Tbest;

//				nextNDT.Clear();
				//bMatchOK = (countGood > 8 && countGood * 3 > countSourceNDT);
				if (countGood < 8)
					bMatchOK = false;
				else if (countGood < 50)
					bMatchOK = (countGood * 2.5  > countSourceNDT /*/ 2*/);
				else
					bMatchOK = (countGood > countSourceNDT / 3);

				nCountGood = countGood;
				nSourceNDT = countSourceNDT;

				return bMatchOK;
			}

			HandleSpecialCase(H, score_gradient_2d.norm());

			// 计算位姿变化向量
			pose_increment_v = -H.ldlt().solve(scg);        // 此为最佳梯度方向
			double dginit = pose_increment_v.dot(scg);

			// 在此方向上不能再降低了，结束迭代
			if (dginit > 0)
			{
				if (score_here > score_best)
					T = Tbest;

//				nextNDT.Clear();
				//bMatchOK = (countGood > 8 && countGood * 3 > countSourceNDT);
				if (countGood < 8)
					bMatchOK = false;
				else if (countGood < 50)
					bMatchOK = (countGood * 2.5  > countSourceNDT /*/ 2*/);
				else
					bMatchOK = (countGood > countSourceNDT / 3);

				nCountGood = countGood;
				nSourceNDT = countSourceNDT;

				return (bMatchOK);
			}

			d2 = getDoubleTime();

			// 在此计算沿最佳梯度下降方向上的最佳移动量
			double step_size = lineSearch2D(pose_increment_v, nextNDT, targetNDT);
			t3 += getDoubleTime() - d2;

			// 计算沿最佳梯度方向上的移动向量
			pose_increment_v = step_size * pose_increment_v;

			// 得到姿态变换矩阵TR
			TR = Eigen::Translation<double, 3>(pose_increment_v(0), pose_increment_v(1), 0) *
				Eigen::AngleAxis<double>(pose_increment_v(2), Eigen::Vector3d::UnitZ());

			// 更新T矩阵
			T = TR * T;

			d2 = getDoubleTime();

			// 对局部NDT图进行TR变换
			nextNDT.Transform(TR);

			t4 += getDoubleTime() - d2;

			// 判断是否已收敛
			if (itr_ctr > 0)
				convergence = ((pose_increment_v.norm()) < DELTA_SCORE);

			// 判断循环次数是否超限
			if (itr_ctr > ITR_MAX)
			{
				convergence = true;
				ret = false;
			}

			matchCycles++;
			itr_ctr++;
		}

		double d2 = getDoubleTime();

		// 取得当前的得分
		score_gradient_2d.setZero();
		double score_here = derivativesNDT_2d(nextNDT, targetNDT, score_gradient_2d, H, false);
		
		t5 += getDoubleTime() - d2;

		// 如果得分不如“最佳得分”，将T取“最佳变换”
		if (score_here > score_best)
			T = Tbest;

		// 释放nextNDT向量
//		nextNDT.Clear();

		runTime = (int)((getDoubleTime() - d1) * 1000);
		int nRunTime2 = (int)(runTime2 * 1000);

		if (ret)
			finalscore = fabs(score_best);
	
		//bMatchOK = (countGood > 8 && countGood * 3 > countSourceNDT);
		if (countGood < 8)
			bMatchOK = false;
		else if (countGood < 50)
			bMatchOK = (countGood * 2.5 > countSourceNDT /*/ 2*/);
		else
			bMatchOK = (countGood > countSourceNDT / 3);

		nCountGood = countGood;
		nSourceNDT = countSourceNDT;

		ret = bMatchOK;
		return ret;
	}

	//iteratively update the score gradient and hessian (2d version)
	// 更新梯度变化得分和Henssian矩阵
	bool NDTMatcherD2D_2D::update_gradient_hessian_local_2d(
		Eigen::MatrixXd &score_gradient,
		Eigen::MatrixXd &Hessian,
		const Eigen::Vector3d & x,
		const Eigen::Matrix3d & B,
		const double &likelihood,
		const Eigen::Matrix<double, 3, 3> &_Jest,
		const Eigen::Matrix<double, 9, 3> &_Hest,
		const Eigen::Matrix<double, 3, 9> &_Zest,
		const Eigen::Matrix<double, 9, 9> &_ZHest,
		bool computeHessian)
	{
		//vars for gradient
		Eigen::Matrix<double, 3, 1> _xtBJ, _xtBZBx, _Q;
		//vars for hessian
		Eigen::Matrix<double, 3, 3> _xtBZBJ, _xtBH, _xtBZBZBx, _xtBZhBx;
		Eigen::Matrix<double, 1, 3> _TMP1, _xtB;

		_xtBJ.setZero();
		_xtBZBx.setZero();
		_Q.setZero();
		_xtBZBJ.setZero();
		_xtBH.setZero();
		_xtBZBZBx.setZero();
		_xtBZhBx.setZero();
		_TMP1.setZero();
		_xtB.setZero();

		_xtB = x.transpose()*B;
		_xtBJ = _xtB*_Jest;

		//    for(unsigned int i=0; i<3; i++)
		//    {
		_TMP1 = _xtB*_Zest.block<3, 3>(0, 6)*B;
		_xtBZBx(2) = _TMP1*x;
		if (computeHessian)
		{
			_xtBZBJ.col(2) = (_TMP1*_Jest).transpose(); //-
			for (unsigned int j = 0; j<3; j++)
			{
				_xtBH(2, j) = _xtB*_Hest.block<3, 1>(6, j);
				_xtBZBZBx(2, j) = _TMP1*_Zest.block<3, 3>(0, 3 * j)*B*x;
				_xtBZhBx(2, j) = _xtB*_ZHest.block<3, 3>(6, 3 * j)*B*x;
			}
		}
		//    }
		_Q = 2 * _xtBJ - _xtBZBx;
		double factor = -(lfd2 / 2)*likelihood;
		score_gradient += _Q*factor;

		if (computeHessian)
		{
			Hessian += factor*(2 * _Jest.transpose()*B*_Jest + 2 * _xtBH - _xtBZhBx - 2 * _xtBZBJ.transpose()
				- 2 * _xtBZBJ + _xtBZBZBx + _xtBZBZBx.transpose() - lfd2*_Q*_Q.transpose() / 2); // + Eigen::Matrix<double,6,6>::Identity();

		}
		return true;
	}


	bool NDTMatcherD2D_2D::update_gradient_hessian_2d(
		Eigen::MatrixXd &score_gradient,
		Eigen::MatrixXd &Hessian,
		const Eigen::Vector3d & x,
		const Eigen::Matrix3d & B,
		const double &likelihood,
		bool computeHessian)
	{

		xtBJ.setZero();
		xtBZBx.setZero();
		Q.setZero();
		JtBJ.setZero();
		xtBZBJ.setZero();
		xtBH.setZero();
		xtBZBZBx.setZero();
		xtBZhBx.setZero();
		TMP1.setZero();
		xtB.setZero();

		xtB = x.transpose()*B;
		xtBJ = xtB*Jest;

		TMP1 = xtB*Zest.block<3, 3>(0, 6)*B;
		xtBZBx(2) = TMP1*x;
		if (computeHessian)
		{
			xtBZBJ.col(2) = (TMP1*Jest).transpose(); //-
			for (unsigned int j = 0; j<3; j++)
			{
				xtBH(2, j) = xtB*Hest.block<3, 1>(6, j);
				xtBZBZBx(2, j) = TMP1*Zest.block<3, 3>(0, 3 * j)*B*x;
				xtBZhBx(2, j) = xtB*ZHest.block<3, 3>(6, 3 * j)*B*x;
			}
		}
		Q = 2 * xtBJ - xtBZBx;
		double factor = -(lfd2 / 2)*likelihood;
		score_gradient += Q*factor;

		if (computeHessian)
		{
			Hessian += factor*(2 * Jest.transpose()*B*Jest + 2 * xtBH - xtBZhBx - 2 * xtBZBJ.transpose()
				- 2 * xtBZBJ + xtBZBZBx + xtBZBZBx.transpose() - lfd2*Q*Q.transpose() / 2);

		}
		return true;
	}

	//pre-computes the derivative matrices Jest, Hest, Zest, ZHest
	void NDTMatcherD2D_2D::computeDerivativesLocal_2d(Eigen::Vector3d &x, Eigen::Matrix3d C1,
		Eigen::Matrix<double, 3, 3> &_Jest,
		Eigen::Matrix<double, 9, 3> &_Hest,
		Eigen::Matrix<double, 3, 9> &_Zest,
		Eigen::Matrix<double, 9, 9> &_ZHest,
		bool computeHessian)
	{
		_Jest(0, 2) = -x(1);
		_Jest(1, 2) = x(0);

		Eigen::Matrix3d myBlock;
		//_Zest
		_Zest.block<3, 3>(0, 6) <<
			-2 * C1(0, 1), -C1(1, 1) + C1(0, 0), -C1(1, 2),
			-C1(1, 1) + C1(0, 0), 2 * C1(0, 1), C1(0, 2),
			-C1(1, 2), C1(0, 2), 0;

		if (computeHessian)
		{
			_Hest.block<3, 1>(6, 2) << -x(0), -x(1), 0;
			_ZHest.block<3, 3>(6, 6) <<
				2 * C1(1, 1) - 2 * C1(0, 0), -4 * C1(0, 1), -C1(0, 2),
				-4 * C1(0, 1), 2 * C1(0, 0) - 2 * C1(1, 1), -C1(1, 2),
				-C1(0, 2), -C1(1, 2), 0;
		}
	}

	void NDTMatcherD2D_2D::computeDerivatives_2d(Eigen::Vector3d &x, Eigen::Matrix3d C1, bool computeHessian)
	{
		Jest(0, 2) = -x(1);
		Jest(1, 2) = x(0);

		Eigen::Matrix3d myBlock;
		//_Zest
		Zest.block<3, 3>(0, 6) <<
			-2 * C1(0, 1), -C1(1, 1) + C1(0, 0), -C1(1, 2),
			-C1(1, 1) + C1(0, 0), 2 * C1(0, 1), C1(0, 2),
			-C1(1, 2), C1(0, 2), 0;

		if (computeHessian)
		{
			Hest.block<3, 1>(6, 2) << -x(0), -x(1), 0;
			ZHest.block<3, 3>(6, 6) <<
				2 * C1(1, 1) - 2 * C1(0, 0), -4 * C1(0, 1), -C1(0, 2),
				-4 * C1(0, 1), 2 * C1(0, 0) - 2 * C1(1, 1), -C1(1, 2),
				-C1(0, 2), -C1(1, 2), 0;
		}
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//#define USE_OMP_NDT_MATCHER_D2D_2D
	//compute the score gradient of a point cloud + transformation to an NDT
	double NDTMatcherD2D_2D::derivativesNDT_2d(const VectNDTCells &sourceNDT, /*const */NDTMap &targetNDT,
		Eigen::MatrixXd &score_gradient,	Eigen::MatrixXd &Hessian, bool computeHessian)
	{
		countGood = 0;       // 统计sourceNDT中较好匹配单元的数量
		countSourceNDT = sourceNDT.size();

		targetNDT.ClearMatchStatus();

		//    struct timeval tv_start, tv_end;
		double t1 = getDoubleTime();
		double score_here = 0;
		int n_dimensions = score_gradient.rows();

		//    gettimeofday(&tv_start,NULL);
		score_gradient.setZero();
		Hessian.setZero();

#ifdef USE_OMP_NDT_MATCHER_D2D_2D
		Eigen::MatrixXd score_gradient_omp;
		Eigen::MatrixXd score_here_omp;
		Eigen::MatrixXd Hessian_omp;

#define N_THREADS_2D 6

		//n_threads = omp_get_num_threads();
		score_gradient_omp.resize(n_dimensions, N_THREADS_2D);
		score_here_omp.resize(1, N_THREADS_2D);
		Hessian_omp.resize(n_dimensions, n_dimensions*N_THREADS_2D);

		score_gradient_omp.setZero();
		score_here_omp.setZero();
		Hessian_omp.setZero();
		//std::cout<<n_threads<<" "<<omp_get_thread_num()<<std::endl;

#pragma omp parallel num_threads(N_THREADS_2D)
		{
#pragma omp for
			for (int i = 0; i < sourceNDT.size(); i++)
			{
				bool good = false;

				pcl::PointXYZ point;
				Eigen::Vector3d transformed;
				Eigen::Vector3d meanMoving, meanFixed;
				Eigen::Matrix3d CMoving, CFixed, CSum, Cinv, R;
				Eigen::MatrixXd score_gradient_omp_loc(n_dimensions, 1);
				Eigen::MatrixXd Hessian_omp_loc(n_dimensions, n_dimensions);
				Eigen::Matrix<double, 3, 3> _Jest;
				Eigen::Matrix<double, 9, 3> _Hest;
				Eigen::Matrix<double, 3, 9> _Zest;
				Eigen::Matrix<double, 9, 9> _ZHest;
				double score_here_loc = 0;
				int thread_id = omp_get_thread_num();
				NDTCell *cell;
				bool exists = false;
				double det = 0;


				score_gradient_omp_loc.setZero();
				Hessian_omp_loc.setZero();
				_Jest.setZero();
				_Jest.block<2, 2>(0, 0).setIdentity();
				_Hest.setZero();
				_Zest.setZero();
				_ZHest.setZero();

				meanMoving = sourceNDT[i]->getMean();
				CMoving = sourceNDT[i]->getCov();
				computeDerivativesLocal_2d(meanMoving, CMoving, _Jest, _Hest, _Zest, _ZHest, computeHessian);

				point.x = meanMoving(0);
				point.y = meanMoving(1);
				point.z = meanMoving(2);
				
				std::vector<NDTCell*> cells = targetNDT.getCellsForPoint(point, n_neighbours);
				for (unsigned int j = 0; j < cells.size(); j++)
				{
					cell = cells[j];
					if (cell == NULL)
					{
						continue;
					}
					if (cell->hasGaussian_)
					{
						transformed = meanMoving - cell->getMean();
						CFixed = cell->getCov();
						CSum = (CFixed + CMoving);
						CSum.computeInverseAndDetWithCheck(Cinv, det, exists);
						if (!exists)
							continue;

						double l = (transformed).dot(Cinv*(transformed));
						if (l * 0 != 0)
							continue;

						//if(l > 120) continue;
						double sh = -lfd1*(exp(-lfd2*l / 2));
						if (!update_gradient_hessian_local_2d(score_gradient_omp_loc, Hessian_omp_loc, transformed, Cinv, sh,
							_Jest, _Hest, _Zest, _ZHest, computeHessian))
							continue;

						score_here_loc += sh;

						if (fabs(sh) > 0.85)
						{
							cell->matchStatus = 1;
							if (!good)
								good = true;
						}

						cell = NULL;
					}
				}

				//score_gradient_omp.block(0,thread_id,n_dimensions,1) += score_gradient_omp_loc;
				score_gradient_omp.col(thread_id) += score_gradient_omp_loc;
				Hessian_omp.block(0, n_dimensions*thread_id, n_dimensions, n_dimensions) += Hessian_omp_loc;
				score_here_omp(0, thread_id) += score_here_loc;

				if (good)
					countGood++;
			}
		} //end pragma block

		  //std::cout<<"sgomp: "<<score_gradient_omp<<std::endl;
		  //std::cout<<"somp: "<<score_here_omp<<std::endl;

		score_gradient = score_gradient_omp.rowwise().sum();
		score_here = score_here_omp.sum();
		if (computeHessian)
		{
			//std::cout<<"Homp: "<<Hessian_omp<<std::endl;
			for (int i = 0; i<N_THREADS_2D; ++i)
			{
				Hessian += Hessian_omp.block(0, n_dimensions*i, n_dimensions, n_dimensions);
			}
		}

#else
		pcl::PointXYZ point;
		Eigen::Vector3d transformed;
		Eigen::Vector3d meanMoving, meanFixed;
		Eigen::Matrix3d CMoving, CFixed, CSum, Cinv, R;
		NDTCell *cell;

		bool exists = false;
		double det = 0;
	
		// 依次考察所有的源(当前)NDT单元
		for (unsigned int i = 0; i < sourceNDT.size(); i++)
		{
			bool good = false;

			meanMoving = sourceNDT[i]->getMean();      // 该局部NDT单元的中心点位置
			CMoving = sourceNDT[i]->getCov();          // 该局部NDT单元的协方差矩阵

			// 计算该局部NDT单元的各阶导数
			computeDerivatives_2d(meanMoving, CMoving, computeHessian);

			point.x = meanMoving(0);
			point.y = meanMoving(1);
			point.z = meanMoving(2);

			// 从NDT模型内，提取在该局部点附近指定距离以内的所有NDT单元
			std::vector<NDTCell*> cells = targetNDT.getCellsForPoint(point, n_neighbours);

			// 统计匹配得分
			for (int j = 0; j < cells.size(); j++)
			{
				cell = cells[j];
				if (cell != NULL && cell->hasGaussian_)
				{
					// 计算该局部NDT单元与该模型NDT单元之间的距离差
					transformed = meanMoving - cell->getMean();

					// 取得该模型NDT单元的协方差矩阵
					CFixed = cell->getCov();

					// CSum为局部单元和模型单元NDT单元协方差矩阵之和
					CSum = (CFixed + CMoving);

					// 计算CSum的逆阵CInv
					CSum.computeInverseAndDetWithCheck(Cinv, det, exists);
					if (!exists)
						continue;

					// 计算方差值?
					double l = (transformed).dot(Cinv*(transformed));
					if (l * 0 != 0)
						continue;

					// 计算似然度?
					double sh = -lfd1 * (exp(-lfd2*l / 2));

					// 计算该单元的得分值score_gradient
					if (!update_gradient_hessian_2d(score_gradient, Hessian, transformed, Cinv, sh, computeHessian))
						continue;

					// 累计得分
					score_here += sh;

					if (fabs(sh) > 0.85)
					{
						cell->matchStatus = 1;
						if (!good)
							good = true;
					}

					cell = NULL;
				}
			}
			if (good)
				countGood++;
		}
#endif

		double t2 = getDoubleTime();
		return score_here;
	}

	// perform line search to find the best descent rate (Mohre&Thuente)
	// adapted from NOX
	double NDTMatcherD2D_2D::lineSearch2D(Eigen::Matrix<double, 3, 1> &increment,
		VectNDTCells &sourceNDT, NDTMap &targetNDT)
	{
		// 缺省参数
		double stp = 1.0; //default step
		double recoverystep = 0.01;
		double dginit = 0.0;
		double ftol = 0.11111; //epsilon 1
		double gtol = 0.99999; //epsilon 2
		double stpmax = 4.0;
		double stpmin = 0.0001;
		int maxfev = 40; //max function evaluations
		double xtol = 0.01; //window of uncertainty around the optimal step

		// 局部变量
		VectNDTCells sourceNDTHere;
		double score_init = 0.0;

		Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> ps;
		ps.setIdentity();

		Eigen::Matrix<double, 3, 1> scg_here;
		Eigen::MatrixXd pincr(3, 1), score_gradient_here(3, 1);
		Eigen::MatrixXd pseudoH(3, 3);
		Eigen::Vector3d eulerAngles;

		int info = 0;			// return code
		int infoc = 1;		// return code for subroutine cstep

		// Compute the initial gradient in the search direction and check
		// that s is a descent direction.

		//we want to maximize s, so we should minimize -s
		//score_init = scoreNDT(sourceNDT,targetNDT);

		//gradient directions are opposite for the negated function
		//score_gradient_init = -score_gradient_init;

		score_gradient_here.setZero();
		
		double t1 = getDoubleTime();
		
		score_init = derivativesNDT_2d(sourceNDT, targetNDT, score_gradient_here, pseudoH, false);
		
		runTime2 += getDoubleTime() - t1;
		
		scg_here = score_gradient_here;

		dginit = increment.dot(scg_here);

		// 如果方向错误(梯度上升方向)
		if (dginit >= 0.0)
		{
			increment = -increment;
			dginit = -dginit;

			if (dginit >= 0.0)
			{
				sourceNDTHere.Clear();
				return recoverystep;
			}
		}

		// Initialize local variables.
		bool brackt = false;		// has the soln been bracketed?
		bool stage1 = true;		// are we in stage 1?
		int nfev = 0;			// number of function evaluations
		double dgtest = ftol * dginit; // f for curvature condition
		double width = stpmax - stpmin; // interval width
		double width1 = 2 * width;	// ???

		// initial function value
		double finit = 0.0;
		finit = score_init;

		// The variables stx, fx, dgx contain the values of the step,
		// function, and directional derivative at the best step.  The
		// variables sty, fy, dgy contain the value of the step, function,
		// and derivative at the other endpoint of the interval of
		// uncertainty.  The variables stp, f, dg contain the values of the
		// step, function, and derivative at the current step.

		double stx = 0.0;
		double fx = finit;
		double dgx = dginit;
		double sty = 0.0;
		double fy = finit;
		double dgy = dginit;

		// Start of iteration.
		double stmin, stmax;
		double fm, fxm, fym, dgm, dgxm, dgym;

		while (1)
		{
			// Set the minimum and maximum steps to correspond to the present
			// interval of uncertainty.
			if (brackt)
			{
				stmin = MoreThuente_min(stx, sty);
				stmax = MoreThuente_max(stx, sty);
			}
			else
			{
				stmin = stx;
				stmax = stp + 4 * (stp - stx);
			}

			// Force the step to be within the bounds stpmax and stpmin.
			stp = MoreThuente_max(stp, stpmin);
			stp = MoreThuente_min(stp, stpmax);

			// If an unusual termination is to occur then let stp be the
			// lowest point obtained so far.

			if ((brackt && ((stp <= stmin) || (stp >= stmax))) ||
				(nfev >= maxfev - 1) || (infoc == 0) ||
				(brackt && (stmax - stmin <= xtol * stmax)))
			{
				stp = stx;
			}

			// Evaluate the function and gradient at stp
			// and compute the directional derivative.
			///////////////////////////////////////////////////////////////////////////

			pincr = stp * increment;

			ps = Eigen::Translation<double, 3>(pincr(0), pincr(1), 0)*
				Eigen::AngleAxisd(pincr(2), Eigen::Vector3d::UnitZ());

			sourceNDTHere.Copy(sourceNDT);
			sourceNDTHere.Transform(ps);

			double f = 0.0;
			score_gradient_here.setZero();

			t1 = getDoubleTime();
			
			f = derivativesNDT_2d(sourceNDTHere, targetNDT, score_gradient_here, pseudoH, false);
			
			runTime2 += getDoubleTime() - t1;

			double dg = 0.0;
			scg_here = score_gradient_here;
			dg = increment.dot(scg_here);
			nfev++;

			///////////////////////////////////////////////////////////////////////////

			// Armijo-Goldstein sufficient decrease
			double ftest1 = finit + stp * dgtest;

			// Test for convergence.
			if ((brackt && ((stp <= stmin) || (stp >= stmax))) || (infoc == 0))
				info = 6;			// Rounding errors

			if ((stp == stpmax) && (f <= ftest1) && (dg <= dgtest))
				info = 5;			// stp=stpmax

			if ((stp == stpmin) && ((f > ftest1) || (dg >= dgtest)))
				info = 4;			// stp=stpmin

			if (nfev >= maxfev)
				info = 3;			// max'd out on fevals

			if (brackt && (stmax - stmin <= xtol*stmax))
				info = 2;			// bracketed soln

										// RPP sufficient decrease test can be different
			bool sufficientDecreaseTest = false;
			sufficientDecreaseTest = (f <= ftest1);  // Armijo-Golstein

																  //cout<<"ftest2 "<<gtol*(-dginit)<<endl;
																  //cout<<"sufficientDecrease? "<<sufficientDecreaseTest<<endl;
																  //cout<<"curvature ok? "<<(fabs(dg) <= gtol*(-dginit))<<endl;
			if ((sufficientDecreaseTest) && (fabs(dg) <= gtol*(-dginit)))
				info = 1;			// Success!!!!

			if (info != 0) 		// Line search is done
			{
				if (info != 1) 		// Line search failed
				{
					// RPP add
					// counter.incrementNumFailedLineSearches();

					//if (recoveryStepType == Constant)
					stp = recoverystep;

					//newgrp.computeX(oldgrp, dir, stp);

					//message = "(USING RECOVERY STEP!)";

				}
				else 			// Line search succeeded
				{
					//message = "(STEP ACCEPTED!)";
				}

				// Returning the line search flag
				//cout<<"LineSearch::"<<message<<" info "<<info<<endl;
#if 0
				for (unsigned int i = 0; i<sourceNDTHere.size(); i++)
				{
					if (sourceNDTHere[i] != NULL)
						delete sourceNDTHere[i];
				}
#endif
				sourceNDTHere.Clear();

				//std::cout<<"nfev = "<<nfev<<std::endl;
				return stp;

			} // info != 0

			  // RPP add
			  //counter.incrementNumIterations();

			  // In the first stage we seek a step for which the modified
			  // function has a nonpositive value and nonnegative derivative.

			if (stage1 && (f <= ftest1) && (dg >= MoreThuente_min(ftol, gtol) * dginit))
			{
				stage1 = false;
			}

			// A modified function is used to predict the step only if we have
			// not obtained a step for which the modified function has a
			// nonpositive function value and nonnegative derivative, and if a
			// lower function value has been obtained but the decrease is not
			// sufficient.

			if (stage1 && (f <= fx) && (f > ftest1))
			{

				// Define the modified function and derivative values.

				fm = f - stp * dgtest;
				fxm = fx - stx * dgtest;
				fym = fy - sty * dgtest;
				dgm = dg - dgtest;
				dgxm = dgx - dgtest;
				dgym = dgy - dgtest;

				// Call cstep to update the interval of uncertainty
				// and to compute the new step.

				//VALGRIND_CHECK_VALUE_IS_DEFINED(dgm);
				infoc = MoreThuente_cstep(stx, fxm, dgxm, sty, fym, dgym, stp, fm, dgm,
					brackt, stmin, stmax);

				// Reset the function and gradient values for f.

				fx = fxm + stx*dgtest;
				fy = fym + sty*dgtest;
				dgx = dgxm + dgtest;
				dgy = dgym + dgtest;

			}

			else
			{

				// Call cstep to update the interval of uncertainty
				// and to compute the new step.

				//VALGRIND_CHECK_VALUE_IS_DEFINED(dg);
				infoc = MoreThuente_cstep(stx, fx, dgx, sty, fy, dgy, stp, f, dg,
					brackt, stmin, stmax);

			}

			// Force a sufficient decrease in the size of the
			// interval of uncertainty.

			if (brackt)
			{
				if (fabs(sty - stx) >= 0.66 * width1)
					stp = stx + 0.5 * (sty - stx);
				width1 = width;
				width = fabs(sty - stx);
			}

		} // while-loop

	}

	int NDTMatcherD2D_2D::MoreThuente_cstep(double& stx, double& fx, double& dx,
		double& sty, double& fy, double& dy,
		double& stp, double& fp, double& dp,
		bool& brackt, double stmin, double stmax)
	{
		int info = 0;

		// Check the input parameters for errors.

		if ((brackt && ((stp <= MoreThuente_min(stx, sty)) || (stp >= MoreThuente_max(stx, sty)))) ||
			(dx * (stp - stx) >= 0.0) || (stmax < stmin))
			return info;

		// Determine if the derivatives have opposite sign.

		double sgnd = dp * (dx / fabs(dx));

		// First case. A higher function value.  The minimum is
		// bracketed. If the cubic step is closer to stx than the quadratic
		// step, the cubic step is taken, else the average of the cubic and
		// quadratic steps is taken.

		bool bound;
		double theta;
		double s;
		double gamma;
		double p, q, r;
		double stpc, stpq, stpf;

		if (fp > fx)
		{
			info = 1;
			bound = 1;
			theta = 3 * (fx - fp) / (stp - stx) + dx + dp;
			//VALGRIND_CHECK_VALUE_IS_DEFINED(theta);
			//VALGRIND_CHECK_VALUE_IS_DEFINED(dx);
			//VALGRIND_CHECK_VALUE_IS_DEFINED(dp);
			s = MoreThuente_absmax(theta, dx, dp);
			gamma = s * sqrt(((theta / s) * (theta / s)) - (dx / s) * (dp / s));
			if (stp < stx)
				gamma = -gamma;

			p = (gamma - dx) + theta;
			q = ((gamma - dx) + gamma) + dp;
			r = p / q;
			stpc = stx + r * (stp - stx);
			stpq = stx + ((dx / ((fx - fp) / (stp - stx) + dx)) / 2) * (stp - stx);
			if (fabs(stpc - stx) < fabs(stpq - stx))
				stpf = stpc;
			else
				stpf = stpc + (stpq - stpc) / 2;

			brackt = true;
		}

		// Second case. A lower function value and derivatives of opposite
		// sign. The minimum is bracketed. If the cubic step is closer to
		// stx than the quadratic (secant) step, the cubic step is taken,
		// else the quadratic step is taken.

		else if (sgnd < 0.0)
		{
			info = 2;
			bound = false;
			theta = 3 * (fx - fp) / (stp - stx) + dx + dp;
			s = MoreThuente_absmax(theta, dx, dp);
			gamma = s * sqrt(((theta / s) * (theta / s)) - (dx / s) * (dp / s));
			if (stp > stx)
				gamma = -gamma;
			p = (gamma - dp) + theta;
			q = ((gamma - dp) + gamma) + dx;
			r = p / q;
			stpc = stp + r * (stx - stp);
			stpq = stp + (dp / (dp - dx)) * (stx - stp);
			if (fabs(stpc - stp) > fabs(stpq - stp))
				stpf = stpc;
			else
				stpf = stpq;
			brackt = true;
		}

		// Third case. A lower function value, derivatives of the same sign,
		// and the magnitude of the derivative decreases.  The cubic step is
		// only used if the cubic tends to infinity in the direction of the
		// step or if the minimum of the cubic is beyond stp. Otherwise the
		// cubic step is defined to be either stmin or stmax. The
		// quadratic (secant) step is also computed and if the minimum is
		// bracketed then the the step closest to stx is taken, else the
		// step farthest away is taken.

		else if (fabs(dp) < fabs(dx))
		{
			info = 3;
			bound = true;
			theta = 3 * (fx - fp) / (stp - stx) + dx + dp;
			s = MoreThuente_absmax(theta, dx, dp);

			// The case gamma = 0 only arises if the cubic does not tend
			// to infinity in the direction of the step.

			gamma = s * sqrt(MoreThuente_max(0, (theta / s) * (theta / s) - (dx / s) * (dp / s)));
			if (stp > stx)
				gamma = -gamma;

			p = (gamma - dp) + theta;
			q = (gamma + (dx - dp)) + gamma;
			r = p / q;
			if ((r < 0.0) && (gamma != 0.0))
				stpc = stp + r * (stx - stp);
			else if (stp > stx)
				stpc = stmax;
			else
				stpc = stmin;

			stpq = stp + (dp / (dp - dx)) * (stx - stp);
			if (brackt)
			{
				if (fabs(stp - stpc) < fabs(stp - stpq))
					stpf = stpc;
				else
					stpf = stpq;
			}
			else
			{
				if (fabs(stp - stpc) > fabs(stp - stpq))
					stpf = stpc;
				else
					stpf = stpq;
			}
		}

		// Fourth case. A lower function value, derivatives of the same
		// sign, and the magnitude of the derivative does not decrease. If
		// the minimum is not bracketed, the step is either stmin or
		// stmax, else the cubic step is taken.

		else
		{
			info = 4;
			bound = false;
			if (brackt)
			{
				theta = 3 * (fp - fy) / (sty - stp) + dy + dp;
				s = MoreThuente_absmax(theta, dy, dp);
				gamma = s * sqrt(((theta / s)*(theta / s)) - (dy / s) * (dp / s));
				if (stp > sty)
					gamma = -gamma;
				p = (gamma - dp) + theta;
				q = ((gamma - dp) + gamma) + dy;
				r = p / q;
				stpc = stp + r * (sty - stp);
				stpf = stpc;
			}
			else if (stp > stx)
				stpf = stmax;
			else
				stpf = stmin;
		}

		// Update the interval of uncertainty. This update does not depend
		// on the new step or the case analysis above.

		if (fp > fx)
		{
			sty = stp;
			fy = fp;
			dy = dp;
		}
		else
		{
			if (sgnd < 0.0)
			{
				sty = stx;
				fy = fx;
				dy = dx;
			}
			stx = stp;
			fx = fp;
			dx = dp;
		}

		// Compute the new step and safeguard it.

		stpf = MoreThuente_min(stmax, stpf);
		stpf = MoreThuente_max(stmin, stpf);
		stp = stpf;
		if (brackt && bound)
		{
			if (sty > stx)
				stp = MoreThuente_min(stx + 0.66 * (sty - stx), stp);
			else
				stp = MoreThuente_max(stx + 0.66 * (sty - stx), stp);
		}

		return info;

	}

	double NDTMatcherD2D_2D::MoreThuente_min(double a, double b)
	{
		return (a < b ? a : b);
	}

	double NDTMatcherD2D_2D::MoreThuente_max(double a, double b)
	{
		return (a > b ? a : b);
	}

	double NDTMatcherD2D_2D::MoreThuente_absmax(double a, double b, double c)
	{
		a = fabs(a);
		b = fabs(b);
		c = fabs(c);

		if (a > b)
			return (a > c) ? a : c;
		else
			return (b > c) ? b : c;
	}

	double NDTMatcherD2D_2D::normalizeAngle(double a)
	{
		//set the angle between -M_PI and M_PI
		return atan2(sin(a), cos(a));
	}

	// 清除中间变量及状态
	void NDTMatcherD2D_2D::ClearTemp()
	{
		nextNDT.Clear();
		nextNDT0.Clear();
	}
}
