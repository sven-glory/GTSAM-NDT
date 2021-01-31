/**
 * @file isam.cpp
 * @brief Main isam program.
 * @author Michael Kaess
 * @version $Id: isam.cpp 6377 2012-03-30 20:06:44Z kaess $
 *
 * Copyright (C) 2009-2013 Massachusetts Institute of Technology.
 * Michael Kaess, Hordur Johannsson, David Rosen,
 * Nicholas Carlevaris-Bianco and John. J. Leonard
 *
 * This file is part of iSAM.
 *
 * iSAM is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation; either version 2.1 of the License, or (at
 * your option) any later version.
 *
 * iSAM is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 * License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with iSAM.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <iostream> 

const std::string usage = "\n"
    "Usage:\n"
    "  iSAM [OPTION...] FILE\n"
    "\n"
    "Options:\n"
    "  -h  -?       show help options\n"
    "  -v           verbose - additional output\n"
    "  -q           quiet - no output\n"
    "  -n <number>  max. number of lines to read, 0=all\n"
    "  -G           GUI: show in 3D viewer\n"
    "  -L           LCM: send data to external process\n"
    "  -S [fname]   save statistics\n"
    "  -W [fname]   write out final result\n"
    "  -F           force use of numerical derivatives\n"
    "  -C           calculate marginal covariances\n"
    "  -B           batch processing\n"
    "  -M           use Levenberg-Marquardt for batch\n"
    "  -P           use Powell's Dog-Leg algorithm for optimization\n"
    "  -N           no optimization\n"
    "  -R           use robust (pseudo-Huber) cost function\n"
    "  -d <number>  #steps between drawing/sending data\n"
    "  -u <number>  #steps between any updates (batch or incremental)\n"
    "  -b <number>  #steps between batch steps, 0=never\n"
    "  -s <number>  #steps between solution (backsubstitution)\n"
    "\n";

const std::string intro = "\n"
    "Incremental Smoothing and Mapping (iSAM) version 1.6\n"
    "(C) 2009-2012 Massachusetts Institute of Technology\n"
    "Michael Kaess, Hordur Johannsson, David M. Rosen, and John J. Leonard\n"
    "\n";

#include <stdio.h>
#include <cstring>
#include <map>
#include <Eigen/Dense>

#include <isam/isam.h>
#include <isam/robust.h>

#include "Loader.h"
#ifdef USE_LCM
#include "Lcm.h"
#endif
#ifdef USE_GUI
// needed in file with main() function to work on Mac...
#include "SDL.h"
#include "Viewer.h"
#endif

// for getopt / command line options processing
//#include <unistd.h>
extern int optind;
extern char *optarg;

using namespace std;
using namespace isam;
using namespace Eigen;

const int FNAME_MAX = 500;
char fname[FNAME_MAX];
char fname_stats[FNAME_MAX] = "isam_stats.txt";
char fname_result[FNAME_MAX] = "isam_result.txt";

bool use_gui = false;
bool use_lcm = false;
bool save_stats = false;
bool write_result = false;
bool calculate_covariances = false;
bool batch_processing = false;
bool no_optimization = false;
int parse_num_lines = 0;

// draw state every mod_draw steps
int mod_draw = 1;

Slam slam;
Properties prop;
#ifdef USE_LCM
Lcm lcm;
bool lcm_first = true;
#endif
#ifdef USE_GUI
Viewer viewer;
#endif
Loader* loader;

// for recording of statistics for each time step (for visualization)
class Stats 
{
public:
  double time;
  double chi2;
  unsigned int nnz;
  double local_chi2;
  unsigned int nnodes;
  unsigned int nconstraints;
};
vector<class Stats> stats;

// hard coded for now, affecting all constraints
double robust_cost_function(double d) 
{
  return cost_pseudo_huber(d, .5);
}



/**
 * Calculate covariances for both points and poses up to the given time step.
 */
void covariances(unsigned int step, list<MatrixXd>& point_marginals,
    list<MatrixXd>& pose_marginals) 
{
  // make sure return arguments are empty
  point_marginals.clear();
  pose_marginals.clear();

  // combining everything into one call is faster,
  // as it avoids recalculating commonly needed entries
  Covariances::node_lists_t node_lists;
  for (unsigned int i = 0; i < step; i++) 
  {
    list<Node*> entry;
    entry.push_back(loader->pose_nodes()[i]);
    node_lists.push_back(entry);
  }
  for (unsigned int i = 0; i < loader->num_points(step); i++) 
  {
    list<Node*> entry;
    entry.push_back(loader->point_nodes()[i]);
    node_lists.push_back(entry);
  }
  pose_marginals = slam.covariances().marginal(node_lists);

  // split into points and poses
  if (pose_marginals.size() > 0) 
  {
    list<MatrixXd>::iterator center = pose_marginals.begin();
    for (unsigned int i = 0; i < step; i++, center++)
      ;
    point_marginals.splice(point_marginals.begin(), pose_marginals, center,
        pose_marginals.end());
  }
}

/**
 * Visualize data during optimization in internal viewer
 * or send data via LCM (to an external viewer).
 */
void visualize(unsigned int step) 
{
  list<MatrixXd> point_marginals;
  list<MatrixXd> pose_marginals;
  if (calculate_covariances && (use_gui || use_lcm) && (step % mod_draw == 0)) 
  {
    covariances(step, point_marginals, pose_marginals);
  }

#ifdef USE_LCM
  {
    // ids also determine color in viewer
    const int id_trajectory = 0;
    const int id_landmarks = 1;
    const int id_constraints = 2;
    const int id_measurements = 3;
    const int id_pose_covs = 4;
    const int id_point_covs = 5;

    // send to LCM viewer
    if (use_lcm && lcm_first) 
	{
      lcm.send_reset();
      lcm_first = false;
    }
    if (use_lcm && step%mod_draw==0) 
	{
      lcm.send_nodes(loader->poses(step), id_trajectory, (char*)"Trajectory", 1);
      lcm.send_nodes(loader->points(step), id_landmarks, (char*)"Landmarks", loader->is_3d() ? 3 : 2);
      lcm.send_links(loader->constraints(step), id_constraints,
          (char*)"Odometry", id_trajectory, id_trajectory);
      lcm.send_links(loader->measurements(step), id_measurements,
          (char*)"Measurements", id_trajectory, id_landmarks);
      if (calculate_covariances) {
        lcm.send_covariances(pose_marginals, id_pose_covs,
            (char*)"Pose Covs", id_trajectory, loader->is_3d());
        lcm.send_covariances(point_marginals, id_point_covs,
            (char*)"Point Covs", id_landmarks, loader->is_3d());
      }
    }
  }
#endif

#ifdef USE_GUI
  {
    // display in internal 3D viewer
    const int id_trajectory = 0;
    const int id_landmarks = 1;
    const int id_constraints = 2;
    const int id_measurements = 3;
    const int id_pose_covs = 4;
    const int id_point_covs = 5;
    if (use_gui && step%mod_draw==0) 
	{
      viewer.set_nodes(loader->poses(step), id_trajectory,
          (char*)"Trajectory", VIEWER_OBJ_POSE3D);
      viewer.set_nodes(loader->points(step), id_landmarks,
          (char*)"Landmarks", loader->is_3d() ? VIEWER_OBJ_POINT3D : VIEWER_OBJ_TREE);
      viewer.set_links(loader->constraints(step), id_constraints,
          "Odometry", id_trajectory, id_trajectory);
      viewer.set_links(loader->measurements(step), id_measurements,
          "Measurements", id_trajectory, id_landmarks);
      if (calculate_covariances) {
        viewer.set_covariances(pose_marginals, id_pose_covs,
            (char*)"Pose Covs", id_trajectory, loader->is_3d());
        viewer.set_covariances(point_marginals, id_point_covs,
            (char*)"Point Covs", id_landmarks, loader->is_3d());
      }
    }
  }
#endif
}

/**
 * Quit if viewer was closed.
 */
void check_quit() 
{
#ifdef USE_GUI
  if (viewer.exit_requested()) 
  {
    cout << endl << "Aborted by user..." << endl;
    exit(0);
  }
#endif
}

/**
 * Incrementally process factors.
 */
void incremental_slam() 
{
  unsigned int step = 0;

  unsigned int next_step = step;
  // step by step after reading log file
  for (; loader->more_data(&next_step); step = next_step) 
  {
    check_quit();

    double t0 = tic();

    tic("setup");

    // add new variables and constraints for current step
    for (unsigned int s = step; s < next_step; s++) 
	{
  		for (list<Node*>::const_iterator it = loader->nodes(s).begin();
      	it != loader->nodes(s).end(); it++) 
	  	{
    		if (prop.verbose)
      			cout << **it << endl;
   			 slam.add_node(*it);
  		}
        for (list<Factor*>::const_iterator it = loader->factors(s).begin();
          it != loader->factors(s).end(); it++) 
	    {
        	if (prop.verbose)
          		cout << **it << endl;
        	slam.add_factor(*it);
  	    }
    }

    toc("setup");
    tic("incremental");

    if (!(batch_processing || no_optimization)) 
	{
      slam.update();
    }

    toc("incremental");

    if (save_stats) 
	{
      stats.resize(step + 1);
      stats[step].time = toc(t0);
      stats[step].chi2 = slam.normalized_chi2();
      stats[step].nnz = slam.get_R().nnz();
      stats[step].local_chi2 = slam.local_chi2(100); // last 100 constraints
      stats[step].nnodes = slam.get_nodes().size();
      stats[step].nconstraints = slam.get_factors().size();
    }

    // visualization is not counted in timing
    if (!(batch_processing || no_optimization)) 
	{
      visualize(step);
    }
  }

  visualize(step - 1);

  if (!no_optimization)
  {
    if (batch_processing) 
	{
      tic("batch");
      slam.batch_optimization();
      toc("batch");
    } 
	else 
	{
      // end with a batch step/relinearization
      prop.mod_batch = 1;
      slam.set_properties(prop);
      tic("final");
      slam.update();
      toc("final");
    }
  }

  visualize(step - 1);
}

/**
 * The actual processing of data, in separate thread if GUI enabled.
 */
int process(void* unused) {

  // incrementally process data
  slam.set_properties(prop);
  incremental_slam();

  toc("all");

  if (!prop.quiet) {
    if (!batch_processing) {
      cout << endl;
    }
    double accumulated = tictoc("setup") + tictoc("incremental")
        + tictoc("batch");
    cout << "Accumulated computation time: " << accumulated << "s" << endl;
    cout << "(Overall execution time: " << tictoc("all") << "s)" << endl;
    slam.print_stats();
    cout << endl;
  }

  if (save_stats) {
    cout << "Saving statistics to " << fname_stats << endl;
    //save_statistics(fname_stats);
    cout << endl;
  }
  if (write_result) {
    cout << "Saving result to " << fname_result << endl;
    slam.save(fname_result);
    cout << endl;
  }

#ifdef USE_GUI
  if (use_gui) {
    while (true) {
      if (viewer.exit_requested()) {
        exit(0);
      }
      SDL_Delay(100);
    }
  }
#endif

  exit(0);
}

/**
 * Everything starts here.
 */
int main(int argc, char* argv[]) 
{

  tic("all");

  //process_arguments(argc, argv);

  cout << intro;

  if (!prop.quiet) 
  {
    cout << "Reading " << fname;
    if (parse_num_lines > 0) 
	{
      cout << " (only " << parse_num_lines << " lines)";
    }
    cout << endl;
  }
  // parse all data and get into suitable format for incremental processing
  Loader loader_(fname, parse_num_lines, prop.verbose);
  loader = &loader_;
  if (!prop.quiet) 
  {
    loader_.print_stats();
    cout << endl;
  }

  if (!prop.quiet) 
  {
    if (batch_processing) 
	{
      cout << "Performing SAM (batch processing)\n";
    } 
	else 
	{
      cout << "Performing iSAM with parameters:\n";
      cout << "  Draw/send every " << mod_draw << " steps\n";
      cout << "  Update every " << prop.mod_update << " steps\n";
      cout << "  Solve every " << prop.mod_solve << " steps\n";
      cout << "  Batch every " << prop.mod_batch << " steps\n";
    }
    cout << endl;
  }

#ifdef USE_GUI
  if (use_gui) {
    cout << "3D viewer:\n";
    cout << "  Exit: Esc or \"q\"\n";
    cout << "  Reset view: \"r\"\n";
    cout << "  Toggle background color \"c\"\n";
    cout << "  Rotate: left mouse button\n";
    cout << "  Translate: middle mouse button or CTRL+left mouse button\n";
    cout << "  Scale: right mouse button or SHIFT+left mouse button or mouse wheel\n";
    cout << endl;
    // process will be run in separate thread
    viewer.init(process);
  } else {
    process(NULL);
  }
#else
  // no threads needed, simply run process
  process(NULL);
#endif

  return 0;
}
